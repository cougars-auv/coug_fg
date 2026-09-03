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
        self.core = coug_fg_py.FactorGraphPy(config_paths, namespace)
        self.params = self.core.get_params()
        self.namespace = namespace
        self.urdf = urdf

        self.solver_type = coug_fg_py.parse_solver_type(self.params["solver_type"])
        self.is_lm = self.solver_type == SolverType.LEVENBERG_MARQUARDT
        if self.is_lm and not self.params["publish_smoothed_path"]:
            raise RuntimeError(
                "LevenbergMarquardt requires publish_smoothed_path to be set to true."
            )

        self.keyframe_source = coug_fg_py.parse_keyframe_source(
            self.params["keyframe_source"]
        )
        self.backup_keyframe_source = coug_fg_py.parse_keyframe_source(
            self.params["backup_keyframe_source"]
        )

        sensors = self.params["sensors"]
        loose_preint = self.params["comparison"]["enable_loose_dvl_preintegration"]

        gps, depth, mag, ahrs, dvl, wrench = (
            sensors["gps"],
            sensors["depth"],
            sensors["mag"],
            sensors["ahrs"],
            sensors["dvl"],
            sensors["wrench"],
        )

        self.enabled = {
            "imu": True,
            "gps": gps["enable"] or gps["enable_init_priors"],
            "depth": depth["enable"] or depth["enable_init_priors"],
            "mag": mag["enable"],
            "ahrs": ahrs["enable"] or ahrs["enable_init_priors"] or loose_preint,
            "dvl": dvl["enable"] or dvl["enable_init_priors"],
            "wrench": wrench["enable"] or wrench["enable_dropout_only"],
        }

        multiagent = self.params["multiagent"]
        self.multiagent_topics = (
            [f"/{ns}/{multiagent['status_topic']}" for ns in multiagent["namespaces"]]
            if multiagent["enable_multiagent"]
            else []
        )
        self.multiagent_keys = [
            f"multiagent_{i}" for i in range(len(self.multiagent_topics))
        ]

        for source in (self.keyframe_source, self.backup_keyframe_source):
            sensor = SOURCE_SENSORS.get(source)
            if sensor in ("dvl", "depth") and not sensors[sensor]["enable"]:
                raise ValueError(
                    f"Keyframe source '{self.keyframe_source}' or backup "
                    f"'{self.backup_keyframe_source}' references a disabled sensor."
                )

        self.is_initialized = False
        self.results: list[dict] = []

        self.queues: dict[str, list[tuple]] = {
            key: [] for key in (*SENSORS, *self.multiagent_keys)
        }

        self.tfs: dict[str, tuple[np.ndarray, np.ndarray]] = {}

        self._stream_time = 0.0
        self._last_msg_time: dict[str, float] = {}
        self._last_target_time: float | None = None
        self._last_timer_time: float | None = None
        self._rate_limits: dict[str, float] = {}

    @property
    def topic_map(self) -> dict[str, list[str]]:
        sources = [(self.params["topics"][s], s) for s in SENSORS if self.enabled[s]]
        sources += list(zip(self.multiagent_topics, self.multiagent_keys, strict=True))

        prefix = f"/{self.namespace}/" if self.namespace else "/"
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
        self.queues[sensor].append(measurement)
        self._last_msg_time[sensor] = measurement[0]
        if is_neighbor:
            return

        sources = (self.keyframe_source, self.backup_keyframe_source)
        if TRIGGER_SOURCES.get(sensor) in sources:
            self._notify_frontend()
        if KeyframeSource.TIMER in sources:
            self._tick_keyframe_timer()

    def finalize(self) -> None:
        if self.is_lm and self.is_initialized:
            result = self.core.optimize()
            self.results = list(result.get("smoothed_path", [])) if result else []

    def get_results(self) -> dict | None:
        if not self.results:
            return None

        results = {
            k: np.array([r[k] for r in self.results])
            for k in self.results[0]
            if k != "smoothed_path"
        }

        # Offline, pose covariance is just left at the target frame here
        base_pos, base_quat = self.tfs["base"]
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
        if sensor in self.tfs:
            return

        cfg = (
            self.params["multiagent"]
            if sensor == "modem"
            else self.params["sensors"][sensor]
        )
        frame = cfg["parameter_frame"] if cfg["use_parameter_frame"] else frame_id
        self.tfs[sensor] = self._lookup_static_tf(cfg, frame)

    def _lookup_static_tf(self, cfg: dict, frame: str) -> tuple[np.ndarray, np.ndarray]:
        # Offline, transforms come from the URDF instead of a live TF tree
        if cfg["use_parameter_tf"]:
            return np.array(cfg["tf_position"]), np.array(cfg["tf_orientation"])
        if self.urdf is None:
            raise RuntimeError(
                f"use_parameter_tf is false for frame '{frame}' but no URDF was found."
            )
        return self.urdf.lookup(self.params["target_frame"], frame)

    def _drain_all_queues(self) -> dict[str, list]:
        bundle: dict[str, list] = {s: self.queues[s] for s in SENSORS}
        bundle["multiagent"] = [self.queues[k] for k in self.multiagent_keys]
        self.queues = {key: [] for key in self.queues}
        return bundle

    def _restore_all_queues(self, bundle: dict[str, list]) -> None:
        for sensor in SENSORS:
            self.queues[sensor][:0] = bundle[sensor]
        for key, msgs in zip(self.multiagent_keys, bundle["multiagent"], strict=True):
            self.queues[key][:0] = msgs

    def _initialize_graph(self) -> None:
        if "base" not in self.tfs:
            base = self.params["sensors"]["base"]
            self.tfs["base"] = self._lookup_static_tf(base, self.params["base_frame"])

        queues = self._drain_all_queues()

        if self.core.initialize(self._stream_time, queues, self.tfs):
            self.is_initialized = True
            logger.info("Graph initialized successfully.")
        else:
            self._restore_all_queues(queues)

    def _active_keyframe_source(self) -> KeyframeSource:
        if self.keyframe_source == KeyframeSource.TIMER:
            return self.keyframe_source

        sensor = SOURCE_SENSORS.get(
            self.keyframe_source, SOURCE_SENSORS[KeyframeSource.DEPTH]
        )
        last_received = self._last_msg_time.get(sensor)
        newest_stamp = self._last_msg_time.get("imu")

        timed_out = last_received is None or (
            newest_stamp is not None
            and newest_stamp - last_received > self.params["keyframe_timeout_sec"]
        )
        if not timed_out:
            return self.keyframe_source

        if self.backup_keyframe_source == KeyframeSource.NONE:
            logger.error(
                f"Primary keyframe source '{self.keyframe_source}' timed out and no "
                "backup is configured. No new keyframes will be created."
            )
            return self.keyframe_source

        logger.warning(
            f"Primary keyframe source '{self.keyframe_source}' timed out. "
            f"Using backup '{self.backup_keyframe_source}'."
        )
        return self.backup_keyframe_source

    def _update_graph(self) -> None:
        sensor = SOURCE_SENSORS.get(self._active_keyframe_source())
        target_time = (
            self._last_msg_time.get(sensor) if sensor and self.queues[sensor] else None
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

        queues = self._drain_all_queues()
        leftover = self.core.update(target_time, queues, self.tfs)
        self._restore_all_queues(queues if leftover is None else leftover)

    def _notify_frontend(self) -> None:
        if not self.is_initialized:
            self._initialize_graph()
        elif self._check_and_update_rate_limit(
            "update", self.params["max_update_rate_hz"]
        ):
            self._update_graph()
            self._notify_backend()

    def _tick_keyframe_timer(self) -> None:
        period = 1.0 / self.params["keyframe_timer_hz"]
        if self._last_timer_time is None:
            self._last_timer_time = self._stream_time
        elif self._stream_time - self._last_timer_time >= period:
            self._last_timer_time = self._stream_time
            self._notify_frontend()

    def _optimize_graph(self) -> None:
        if result := self.core.optimize():
            new_keyframes = result.pop("new_keyframes")
            if result.pop("processing_overflow"):
                logger.warning(
                    f"Processing overflow. Batching {new_keyframes} keyframes."
                )
            self.results.append(result)

    def _notify_backend(self) -> None:
        # Offline, LevenbergMarquardt batch optimizes once in finalize()
        if self.is_lm:
            return
        if self.is_initialized and self._check_and_update_rate_limit(
            "optimize", self.params["max_opt_rate_hz"]
        ):
            self._optimize_graph()
