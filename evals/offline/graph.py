# Copyright (c) 2026 BYU FROST Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from offline.urdf import UrdfTree

FGO_LIB_PATH = str(
    Path.home()
    / "cougars-dev/ros2_ws/install/coug_fgo/lib"
    / f"python{sys.version_info.major}.{sys.version_info.minor}"
    / "site-packages"
)
sys.path.insert(0, FGO_LIB_PATH)
from typing import ClassVar

import coug_fgo_py

logger = logging.getLogger(__name__)

SENSORS = ("imu", "gps", "depth", "mag", "ahrs", "dvl", "wrench")


class OfflineFactorGraph:
    """
    Offline factor graph using the FactorGraphPy Python wrapper.

    Should match the ROS 2 framework in ``factor_graph.cpp`` as closely as possible.

    :author: Nelson Durrant (w Opus 4.8)
    :date: July 2026
    """

    # IMPORTANT! Offline, the timer fires on message stamps instead of the ROS 2 clock
    SOURCE_SENSORS: ClassVar[dict[str, str]] = {
        "DVL": "dvl",
        "Depth": "depth",
        "Timer": "imu",
    }
    INIT_SENSORS = ("imu", "gps", "depth", "ahrs", "dvl")

    def __init__(
        self,
        config_paths: list[str],
        namespace: str = "",
        urdf: UrdfTree | None = None,
    ):
        """
        Load the parameters and prepare the sensor queues.

        :param config_paths: Parameter YAML files, in increasing priority.
        :param namespace: AUV namespace used to select parameters.
        :param urdf: Parsed URDF tree for transform lookups, if available.
        :raises RuntimeError: If LevenbergMarquardt lacks publish_smoothed_path.
        :raises ValueError: If a keyframe source is unknown or its sensor is disabled.
        """
        # --- Node Settings ---
        self.core = coug_fgo_py.FactorGraphPy(config_paths, namespace)
        self.params = self.core.get_params()
        self.namespace = namespace
        self.urdf = urdf

        self.is_lm = self.params["solver_type"] == "LevenbergMarquardt"
        if self.is_lm and not self.params["publish_smoothed_path"]:
            raise RuntimeError(
                "LevenbergMarquardt requires publish_smoothed_path to be set to true."
            )

        # --- Keyframe Settings ---
        self.keyframe_source = self.params["keyframe_source"]
        self.backup_keyframe_source = self.params["backup_keyframe_source"]

        # --- Sensor Settings ---
        subscribed = {
            key: cfg["enable"] or cfg["enable_extra_only"]
            for key, cfg in self.params["sensors"].items()
        }
        loose_preint = self.params["comparison"]["enable_loose_dvl_preintegration"]

        self.enabled = {
            "imu": True,
            "gps": subscribed["gps"],
            "depth": subscribed["depth"],
            "mag": subscribed["mag"],
            "ahrs": subscribed["ahrs"] or loose_preint,
            "dvl": subscribed["dvl"],
            "wrench": subscribed["dynamics"],
        }

        multiagent = self.params["multiagent"]
        self.multiagent_topics = (
            multiagent["topics"] if multiagent["enable_multiagent"] else []
        )
        self.multiagent_keys = [
            f"multiagent_{i}" for i in range(len(self.multiagent_topics))
        ]

        # Ensure the keyframe sources are valid
        valid_sources = {"None", *self.SOURCE_SENSORS}
        for source in (self.keyframe_source, self.backup_keyframe_source):
            if source not in valid_sources:
                raise ValueError(f"Unknown keyframe source: {source}")
            sensor = self.SOURCE_SENSORS.get(source)
            if sensor in ("dvl", "depth") and not self.enabled[sensor]:
                raise ValueError(
                    f"Keyframe source '{self.keyframe_source}' or backup "
                    f"'{self.backup_keyframe_source}' references a disabled sensor."
                )

        # --- Graph State ---
        self.is_initialized = False
        self.init_data_ready = False
        self.results: list[dict] = []

        self.queues: dict[str, list[tuple]] = {
            key: [] for key in (*SENSORS, *self.multiagent_keys)
        }

        self.target_T_base: tuple[np.ndarray, Rotation] | None = None
        self.tfs_resolved: set[str] = set()

        self._using_backup = False
        self._stream_time = 0.0
        self._last_msg_time: dict[str, float] = {}
        self._last_target_time: float | None = None
        self._last_timer_time: float | None = None
        self._rate_limits: dict[str, float] = {}

    @property
    def topic_map(self) -> dict[str, list[str]]:
        """
        Map each namespace-resolved topic to the sensors that read from it.

        :return: Resolved topic names to lists of sensor keys.
        """
        sources = [(self.params["topics"][s], s) for s in SENSORS if self.enabled[s]]
        sources += list(zip(self.multiagent_topics, self.multiagent_keys))

        prefix = f"/{self.namespace}/" if self.namespace else "/"
        topics: dict[str, list[str]] = {}
        for topic, key in sources:
            resolved = topic if topic.startswith("/") else prefix + topic
            topics.setdefault(resolved, []).append(key)
        return topics

    def pending_init_sensors(self) -> list[str]:
        """
        Return enabled sensors that have not reported yet.

        :return: Sensor keys blocking initialization, in priority order.
        """
        return [
            s
            for s in self.INIT_SENSORS
            if self.enabled[s] and s not in self._last_msg_time
        ]

    def add_message(self, sensor: str, frame_id: str, measurement: tuple) -> None:
        """
        Queue one sensor measurement and trigger keyframe processing when due.

        :param sensor: Sensor key from SENSORS, or ``multiagent_{i}`` for neighbor status.
        :param frame_id: ROS frame the measurement was reported in.
        :param measurement: Extracted measurement tuple, with the stamp first.
        """
        self._stream_time = max(self._stream_time, measurement[0])

        is_neighbor = sensor.startswith("multiagent_")
        self._resolve_sensor_tf("modem" if is_neighbor else sensor, frame_id)
        self.queues[sensor].append(measurement)
        self._last_msg_time[sensor] = measurement[0]
        if is_neighbor:
            return

        sources = (self.keyframe_source, self.backup_keyframe_source)
        if (sensor == "dvl" and "DVL" in sources) or (
            sensor == "depth" and "Depth" in sources
        ):
            self._notify_frontend()
        if "Timer" in sources:
            if self._last_timer_time is None:
                self._last_timer_time = self._stream_time
            elif self._stream_time - self._last_timer_time >= (
                1.0 / self.params["keyframe_timer_hz"]
            ):
                self._last_timer_time = self._stream_time
                self._notify_frontend()

    def finalize(self) -> None:
        """Run the final batch optimization for LevenbergMarquardt solvers."""
        if self.is_lm and self.is_initialized:
            result = self.core.optimize()
            self.results = list(result.get("smoothed_path", [])) if result else []

    def get_results(self) -> dict | None:
        """
        Assemble the optimized results, re-expressed at the base frame.

        :return: Result arrays keyed by state name, or None if there are none.
        """
        if not self.results:
            return None

        results = {
            k: np.array([r[k] for r in self.results])
            for k in self.results[0]
            if k != "smoothed_path"
        }

        # IMPORTANT! Pose covariance is just left at the target frame here
        base_pos, base_rot = self.target_T_base
        target_positions = np.column_stack((results["x"], results["y"], results["z"]))
        target_quats = np.column_stack(
            (results["qx"], results["qy"], results["qz"], results["qw"])
        )

        map_R_target = Rotation.from_quat(target_quats)
        map_R_base = map_R_target * base_rot
        map_t_base = map_R_target.apply(base_pos) + target_positions

        results["x"], results["y"], results["z"] = map_t_base.T
        results["qx"], results["qy"], results["qz"], results["qw"] = (
            map_R_base.as_quat().T
        )
        results["roll"], results["pitch"], results["yaw"] = map_R_base.as_euler("xyz").T

        return results

    def _notify_frontend(self) -> None:
        """Initialize the graph or run a rate-limited update."""
        # IMPORTANT! Offline, the frontend and backend run inline instead of on threads
        if not self.is_initialized:
            self._initialize_graph()
        elif self._check_and_update_rate_limit(
            "update", self.params["max_update_rate_hz"]
        ):
            self._update_graph()
            self._notify_backend()

    def _notify_backend(self) -> None:
        """Run a rate-limited optimization."""
        # IMPORTANT! Offline, LevenbergMarquardt batch optimizes once in finalize()
        if self.is_lm:
            return
        if self.is_initialized and self._check_and_update_rate_limit(
            "optimize", self.params["max_opt_rate_hz"]
        ):
            self._optimize_graph()

    def _initialize_graph(self) -> None:
        """Attempt initialization once every sensor the initializer needs has reported."""
        # --- Wait for Required Sensor Data ---
        if not self.init_data_ready:
            if self.pending_init_sensors():
                return
            self.init_data_ready = True

        # Look up target to base frame TF
        if self.target_T_base is None:
            base = self.params["sensors"]["base"]
            pos, quat = self._load_or_lookup_tf(base, self.params["base_frame"])
            self.core.set_tf("base", pos, quat)
            self.target_T_base = (pos, Rotation.from_quat(quat))

        # --- Compute Initial State ---
        queues = self._drain_all_queues()
        queues.pop("multiagent")  # The state initializer ignores neighbor status

        newest_stamp = max((q[-1][0] for q in queues.values() if q), default=0.0)
        if newest_stamp <= 0.0:
            return

        if self.core.initialize(newest_stamp, **queues):
            self.is_initialized = True
            logger.info("Graph initialized successfully.")

    def _update_graph(self) -> None:
        """Advance the graph to the newest stamp from the active source."""
        active_source = self.keyframe_source
        if active_source != "Timer":
            has_backup = self.backup_keyframe_source != "None"
            last_received = self._last_msg_time.get(
                "dvl" if active_source == "DVL" else "depth"
            )
            newest_stamp = self._last_msg_time.get("imu")
            timed_out = last_received is None or (
                newest_stamp is not None
                and newest_stamp - last_received > self.params["keyframe_timeout_sec"]
            )
            if timed_out:
                if not self._using_backup:
                    if has_backup:
                        logger.warning(
                            f"Primary keyframe source '{self.keyframe_source}' timed out. "
                            f"Using backup '{self.backup_keyframe_source}'."
                        )
                    else:
                        logger.error(
                            f"Primary keyframe source '{self.keyframe_source}' timed out and no backup is configured. "
                            "No new keyframes will be created."
                        )
                if has_backup:
                    active_source = self.backup_keyframe_source
            self._using_backup = timed_out

        # Extract target time
        source = self.SOURCE_SENSORS.get(active_source)
        target_time = (
            self._last_msg_time.get(source) if source and self.queues[source] else None
        )
        if target_time is None or (
            self._last_target_time is not None and target_time <= self._last_target_time
        ):
            return

        min_interval = self.params["min_keyframe_interval_sec"]
        if (
            self._last_target_time is not None
            and target_time - self._last_target_time < min_interval
        ):
            logger.warning(
                f"Keyframe rejected: only {target_time - self._last_target_time:.4f} s "
                f"since the last keyframe (minimum {min_interval:.4f} s)."
            )
            return
        self._last_target_time = target_time

        # --- Update Request ---
        queues = self._drain_all_queues()
        leftover = self.core.update(target_time, **queues)
        self._restore_all_queues(queues if leftover is None else leftover)

    def _optimize_graph(self) -> None:
        """Run one optimization and record the result."""
        # --- Optimization Request ---
        if result := self.core.optimize():
            self.results.append(result)

    def _drain_all_queues(self) -> dict:
        """
        Drain every queue, leaving them all empty.

        :return: Queue bundle keyed by sensor, with neighbor status under ``multiagent``.
        """
        bundle: dict = {s: self.queues[s] for s in SENSORS}
        bundle["multiagent"] = [self.queues[k] for k in self.multiagent_keys]
        self.queues = {key: [] for key in self.queues}
        return bundle

    def _restore_all_queues(self, bundle: dict) -> None:
        """
        Restore unprocessed messages to the front of the queues.

        :param bundle: Queue bundle from ``_drain_all_queues`` or left over from an update.
        """
        for sensor in SENSORS:
            self.queues[sensor][:0] = bundle[sensor]
        for key, msgs in zip(self.multiagent_keys, bundle["multiagent"]):
            self.queues[key][:0] = msgs

    def _check_and_update_rate_limit(self, key: str, max_rate_hz: float) -> bool:
        """
        Check a rate limit against stream time, updating it when passed.

        :param key: Name of the rate-limited action.
        :param max_rate_hz: Maximum rate in Hz, or non-positive to disable the limit.
        :return: True if the action is allowed at the current stream time.
        """
        # IMPORTANT! Offline, rate limits throttle in data time instead of wall time
        if max_rate_hz <= 0.0:
            return True
        last_time = self._rate_limits.get(key)
        if last_time is not None and self._stream_time - last_time < 1.0 / max_rate_hz:
            return False
        self._rate_limits[key] = self._stream_time
        return True

    def _resolve_sensor_tf(self, sensor: str, frame_id: str) -> None:
        """
        Resolve and register a sensor's static transform once.

        :param sensor: Sensor key from SENSORS, or ``modem`` for neighbor status.
        :param frame_id: Frame reported by the sensor's messages.
        """
        if sensor in self.tfs_resolved:
            return
        if sensor == "modem":
            cfg = self.params["multiagent"]
        else:
            cfg = self.params["sensors"]["dynamics" if sensor == "wrench" else sensor]
        frame = cfg["parameter_frame"] if cfg["use_parameter_frame"] else frame_id
        pos, quat = self._load_or_lookup_tf(cfg, frame)
        self.core.set_tf("com" if sensor == "wrench" else sensor, pos, quat)
        self.tfs_resolved.add(sensor)

    def _load_or_lookup_tf(
        self, cfg: dict, frame: str
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Load a transform from parameters or look it up in the URDF.

        :param cfg: Sensor parameter dictionary.
        :param frame: Source frame to look up when parameters are not used.
        :return: Position and xyzw quaternion in the target frame.
        :raises RuntimeError: If a URDF lookup is required but no URDF was found.
        """
        if cfg["use_parameter_tf"]:
            return np.array(cfg["tf_position"]), np.array(cfg["tf_orientation"])
        if self.urdf is None:
            raise RuntimeError(
                f"use_parameter_tf is false for frame '{frame}' but no URDF was found."
            )
        return self.urdf.lookup(self.params["target_frame"], frame)
