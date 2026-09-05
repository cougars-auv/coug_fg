# Copyright 2026 BYU FROST Lab
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
import os
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from offline.urdf import UrdfTree

FG_LIB_PATH = str(
    Path(os.environ["OVERLAY_WS"])
    / "install/coug_fg/lib"
    / f"python{sys.version_info.major}.{sys.version_info.minor}"
    / "site-packages"
)
sys.path.insert(0, FG_LIB_PATH)

import coug_fg_py

logger = logging.getLogger(__name__)

SolverType = coug_fg_py.SolverType
KeyframeSource = coug_fg_py.KeyframeSource

SENSORS = ("imu", "gps", "depth", "mag", "ahrs", "dvl", "wrench")

SOURCE_SENSORS: dict[KeyframeSource, str] = {
    KeyframeSource.DVL: "dvl",
    KeyframeSource.DEPTH: "depth",
    KeyframeSource.TIMER: "imu",
}

TRIGGER_SOURCES: dict[str, KeyframeSource] = {
    "dvl": KeyframeSource.DVL,
    "depth": KeyframeSource.DEPTH,
}


# IMPORTANT! This should match the ROS 2 framework in factor_graph.cpp as closely as possible.
class OfflineFactorGraph:
    def __init__(
        self,
        config_paths: list[str],
        namespace: str = "",
        urdf: UrdfTree | None = None,
    ) -> None:
        self._core = coug_fg_py.FactorGraphPy(config_paths, namespace)
        self._params = self._core.get_params()
        self._namespace = namespace
        self._urdf = urdf

        self._solver_type = coug_fg_py.parse_solver_type(self._params["solver_type"])
        self._is_lm = self._solver_type == SolverType.LEVENBERG_MARQUARDT
        if self._is_lm and not self._params["publish_smoothed_path"]:
            raise RuntimeError(
                "LevenbergMarquardt requires publish_smoothed_path to be set to true."
            )

        self._keyframe_source = coug_fg_py.parse_keyframe_source(
            self._params["keyframe_source"]
        )
        self._backup_keyframe_source = coug_fg_py.parse_keyframe_source(
            self._params["backup_keyframe_source"]
        )

        sensors = self._params["sensors"]
        loose_preint = self._params["comparison"]["enable_loose_dvl_preintegration"]

        gps, depth, mag, ahrs, dvl, wrench = (
            sensors["gps"],
            sensors["depth"],
            sensors["mag"],
            sensors["ahrs"],
            sensors["dvl"],
            sensors["wrench"],
        )

        self._enabled = {
            "imu": True,
            "gps": gps["enable"] or gps["enable_init_priors"],
            "depth": depth["enable"] or depth["enable_init_priors"],
            "mag": mag["enable"],
            "ahrs": ahrs["enable"] or ahrs["enable_init_priors"] or loose_preint,
            "dvl": dvl["enable"] or dvl["enable_init_priors"],
            "wrench": wrench["enable"] or wrench["enable_dropout_only"],
        }

        multiagent = self._params["multiagent"]
        self._multiagent_topics = (
            [f"/{ns}/{multiagent['status_topic']}" for ns in multiagent["namespaces"]]
            if multiagent["enable_multiagent"]
            else []
        )
        self._multiagent_keys = [
            f"multiagent_{i}" for i in range(len(self._multiagent_topics))
        ]

        for source in (self._keyframe_source, self._backup_keyframe_source):
            sensor = SOURCE_SENSORS.get(source)
            if sensor in ("dvl", "depth") and not sensors[sensor]["enable"]:
                raise ValueError(
                    f"Keyframe source '{self._keyframe_source}' or backup "
                    f"'{self._backup_keyframe_source}' references a disabled sensor."
                )

        self.is_initialized = False
        self._results: list[dict] = []

        self._queues: dict[str, list[tuple]] = {
            key: [] for key in (*SENSORS, *self._multiagent_keys)
        }

        self._tfs: dict[str, tuple[np.ndarray, np.ndarray]] = {}

        self._stream_time = 0.0
        self._last_msg_time: dict[str, float] = {}
        self._last_target_time: float | None = None
        self._last_timer_time: float | None = None
        self._rate_limits: dict[str, float] = {}

    @property
    def is_initialized(self) -> bool:
        return self.is_initialized

    @property
    def topic_map(self) -> dict[str, list[str]]:
        sources = [(self._params["topics"][s], s) for s in SENSORS if self._enabled[s]]
        sources += list(
            zip(self._multiagent_topics, self._multiagent_keys, strict=True)
        )

        prefix = f"/{self._namespace}/" if self._namespace else "/"
        topics: dict[str, list[str]] = {}
        for topic, key in sources:
            resolved = topic if topic.startswith("/") else prefix + topic
            topics.setdefault(resolved, []).append(key)
        return topics

    def add_message(self, sensor: str, frame_id: str, measurement: tuple) -> None:
        # Offline, the graph/timer fires on message stamps instead of the wall clock
        self._stream_time = max(self._stream_time, measurement[0])

        is_neighbor = sensor.startswith("multiagent_")
        self._resolve_sensor_tf("modem" if is_neighbor else sensor, frame_id)
        self._queues[sensor].append(measurement)
        self._last_msg_time[sensor] = measurement[0]
        if is_neighbor:
            return

        sources = (self._keyframe_source, self._backup_keyframe_source)
        if TRIGGER_SOURCES.get(sensor) in sources:
            self._notify_frontend()
        if KeyframeSource.TIMER in sources:
            self._tick_keyframe_timer()

    def finalize(self) -> None:
        if self._is_lm and self.is_initialized:
            result = self._core.optimize()
            self._results = list(result.get("smoothed_path", [])) if result else []

    def get_results(self) -> dict | None:
        if not self._results:
            return None

        results = {
            k: np.array([r[k] for r in self._results])
            for k in self._results[0]
            if k != "smoothed_path"
        }

        # Offline, pose covariance is just left at the target frame here
        base_pos, base_quat = self._tfs["base"]
        base_rot = Rotation.from_quat(base_quat)
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

    def _check_and_update_rate_limit(self, key: str, max_rate_hz: float) -> bool:
        if max_rate_hz <= 0.0:
            return True
        last_time = self._rate_limits.get(key)
        if last_time is not None and self._stream_time - last_time < 1.0 / max_rate_hz:
            return False
        self._rate_limits[key] = self._stream_time
        return True

    def _resolve_sensor_tf(self, sensor: str, frame_id: str) -> None:
        if sensor in self._tfs:
            return

        cfg = (
            self._params["multiagent"]
            if sensor == "modem"
            else self._params["sensors"][sensor]
        )
        frame = cfg["parameter_frame"] if cfg["use_parameter_frame"] else frame_id
        self._tfs[sensor] = self._lookup_static_tf(cfg, frame)

    def _lookup_static_tf(self, cfg: dict, frame: str) -> tuple[np.ndarray, np.ndarray]:
        # Offline, transforms come from the URDF instead of a live TF tree
        if cfg["use_parameter_tf"]:
            return np.array(cfg["tf_position"]), np.array(cfg["tf_orientation"])
        if self._urdf is None:
            raise RuntimeError(
                f"use_parameter_tf is false for frame '{frame}' but no URDF was found."
            )
        return self._urdf.lookup(self._params["target_frame"], frame)

    def _drain_all_queues(self) -> dict[str, list]:
        bundle: dict[str, list] = {s: self._queues[s] for s in SENSORS}
        bundle["multiagent"] = [self._queues[k] for k in self._multiagent_keys]
        self._queues = {key: [] for key in self._queues}
        return bundle

    def _restore_all_queues(self, bundle: dict[str, list]) -> None:
        for sensor in SENSORS:
            self._queues[sensor][:0] = bundle[sensor]
        for key, msgs in zip(self._multiagent_keys, bundle["multiagent"], strict=True):
            self._queues[key][:0] = msgs

    def _initialize_graph(self) -> None:
        if "base" not in self._tfs:
            base = self._params["sensors"]["base"]
            self._tfs["base"] = self._lookup_static_tf(base, self._params["base_frame"])

        queues = self._drain_all_queues()

        if self._core.initialize(self._stream_time, queues, self._tfs):
            self.is_initialized = True
            logger.info("Graph initialized successfully.")
        else:
            self._restore_all_queues(queues)

    def _active_keyframe_source(self) -> KeyframeSource:
        if self._keyframe_source == KeyframeSource.TIMER:
            return self._keyframe_source

        sensor = SOURCE_SENSORS.get(
            self._keyframe_source, SOURCE_SENSORS[KeyframeSource.DEPTH]
        )
        last_received = self._last_msg_time.get(sensor)
        newest_stamp = self._last_msg_time.get("imu")

        timed_out = last_received is None or (
            newest_stamp is not None
            and newest_stamp - last_received > self._params["keyframe_timeout_sec"]
        )
        if not timed_out:
            return self._keyframe_source

        if self._backup_keyframe_source == KeyframeSource.NONE:
            logger.error(
                f"Primary keyframe source '{self._keyframe_source}' timed out and no "
                "backup is configured. No new keyframes will be created."
            )
            return self._keyframe_source

        logger.warning(
            f"Primary keyframe source '{self._keyframe_source}' timed out. "
            f"Using backup '{self._backup_keyframe_source}'."
        )
        return self._backup_keyframe_source

    def _update_graph(self) -> None:
        sensor = SOURCE_SENSORS.get(self._active_keyframe_source())
        target_time = (
            self._last_msg_time.get(sensor) if sensor and self._queues[sensor] else None
        )
        if target_time is None or (
            self._last_target_time is not None and target_time <= self._last_target_time
        ):
            return

        min_interval = self._params["min_keyframe_interval_sec"]
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

        queues = self._drain_all_queues()
        leftover = self._core.update(target_time, queues, self._tfs)
        self._restore_all_queues(queues if leftover is None else leftover)

    def _notify_frontend(self) -> None:
        if not self.is_initialized:
            self._initialize_graph()
        elif self._check_and_update_rate_limit(
            "update", self._params["max_update_rate_hz"]
        ):
            self._update_graph()
            self._notify_backend()

    def _tick_keyframe_timer(self) -> None:
        period = 1.0 / self._params["keyframe_timer_hz"]
        if self._last_timer_time is None:
            self._last_timer_time = self._stream_time
        elif self._stream_time - self._last_timer_time >= period:
            self._last_timer_time = self._stream_time
            self._notify_frontend()

    def _optimize_graph(self) -> None:
        if result := self._core.optimize():
            new_keyframes = result.pop("new_keyframes")
            if result.pop("processing_overflow"):
                logger.warning(
                    f"Processing overflow. Batching {new_keyframes} keyframes."
                )
            self._results.append(result)

    def _notify_backend(self) -> None:
        # Offline, LevenbergMarquardt batch optimizes once in finalize()
        if self._is_lm:
            return
        if self.is_initialized and self._check_and_update_rate_limit(
            "optimize", self._params["max_opt_rate_hz"]
        ):
            self._optimize_graph()
