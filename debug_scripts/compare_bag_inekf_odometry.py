#!/usr/bin/env python3
import argparse
import csv
import sys
import sqlite3
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import pinocchio as pin
from inekf import InEKF, Kinematics, NoiseParams, RobotState
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Imu
from unitree_go.msg import LowState


DEFAULT_LOWSTATE_TOPIC = "/lowstate"
DEFAULT_UTLIDAR_IMU_TOPIC = "/utlidar/imu"
DEFAULT_REF_ODOM_TOPIC = "/utlidar/robot_odom"


@dataclass
class OdomSample:
    timestamp_ns: int
    position: np.ndarray
    quaternion_xyzw: np.ndarray


def _try_import_load_go2():
    try:
        from go2_description.loader import loadGo2  # type: ignore

        return loadGo2
    except ModuleNotFoundError:
        candidate_patterns = [
            "/home/*/go2_description/install/go2_description/lib/python*/site-packages",
            "/opt/ros/*/lib/python*/site-packages",
        ]
        for pattern in candidate_patterns:
            for site_pkg in sorted(Path("/").glob(pattern.lstrip("/"))):
                site_pkg_str = str(site_pkg)
                if site_pkg_str not in sys.path:
                    sys.path.append(site_pkg_str)
                try:
                    from go2_description.loader import loadGo2  # type: ignore

                    return loadGo2
                except ModuleNotFoundError:
                    continue
        raise


class OfflineInekf:
    """
    Offline mirror of scripts/inekf_odom.py for bag replay.
    """

    def __init__(
        self,
        imu_source: str,
        robot_freq: float = 500.0,
        gyroscope_noise: float = 0.01,
        accelerometer_noise: float = 0.1,
        gyroscope_bias_noise: float = 0.00001,
        accelerometer_bias_noise: float = 0.0001,
        contact_noise: float = 0.001,
        joint_position_noise: float = 0.001,
        contact_velocity_noise: float = 0.001,
        imu_rotation_rpy: Sequence[float] = (0.0, 0.0, 0.0),
        imu_translation_xyz: Sequence[float] = (0.0, 0.0, 0.0),
        compensate_imu_translation: bool = False,
    ):
        if imu_source not in {"lowstate", "utlidar"}:
            raise ValueError("imu_source must be 'lowstate' or 'utlidar'")
        self.imu_source = imu_source
        self.dt = 1.0 / float(robot_freq)
        self.pause = True

        self.imu_rotation = pin.rpy.rpyToMatrix(np.array(imu_rotation_rpy, dtype=float))
        self.imu_translation = np.array(imu_translation_xyz, dtype=float)
        self.compensate_imu_translation = bool(compensate_imu_translation)
        self.prev_gyro_in_filter_imu: Optional[np.ndarray] = None
        self.prev_imu_stamp_sec: Optional[float] = None
        self.latest_utlidar_gyro: Optional[np.ndarray] = None
        self.latest_utlidar_acc: Optional[np.ndarray] = None
        self.latest_utlidar_stamp_sec: Optional[float] = None

        load_go2 = _try_import_load_go2()
        self.robot = load_go2()
        self.foot_frame_name = [prefix + "_foot" for prefix in ["FL", "FR", "RL", "RR"]]
        self.foot_frame_id = [self.robot.model.getFrameId(frame_name) for frame_name in self.foot_frame_name]

        gravity = np.array([0.0, 0.0, -9.81])
        initial_state = RobotState()
        initial_state.setRotation(np.eye(3))
        initial_state.setVelocity(np.zeros(3))
        initial_state.setPosition(np.zeros(3))
        initial_state.setGyroscopeBias(np.zeros(3))
        initial_state.setAccelerometerBias(np.zeros(3))

        noise_params = NoiseParams()
        noise_params.setGyroscopeNoise(gyroscope_noise)
        noise_params.setAccelerometerNoise(accelerometer_noise)
        noise_params.setGyroscopeBiasNoise(gyroscope_bias_noise)
        noise_params.setAccelerometerBiasNoise(accelerometer_bias_noise)
        noise_params.setContactNoise(contact_noise)

        self.joint_pos_noise = joint_position_noise
        self.contact_vel_noise = contact_velocity_noise

        self.filter = InEKF(initial_state, noise_params)
        self.filter.setGravity(gravity)

    def utlidar_callback(self, msg: Imu):
        self.latest_utlidar_gyro = np.array(
            [float(msg.angular_velocity.x), float(msg.angular_velocity.y), float(msg.angular_velocity.z)]
        )
        self.latest_utlidar_acc = np.array(
            [float(msg.linear_acceleration.x), float(msg.linear_acceleration.y), float(msg.linear_acceleration.z)]
        )
        self.latest_utlidar_stamp_sec = stamp_to_sec(msg.header.stamp)

    def compensate_imu_transform(
        self, gyro: np.ndarray, acc: np.ndarray, stamp_sec: Optional[float]
    ) -> Tuple[np.ndarray, np.ndarray]:
        gyro_in_filter_imu = self.imu_rotation @ gyro
        acc_in_filter_imu = self.imu_rotation @ acc

        if self.compensate_imu_translation:
            gyro_dot = np.zeros(3)
            if self.prev_gyro_in_filter_imu is not None:
                dt = self.dt
                if stamp_sec is not None and self.prev_imu_stamp_sec is not None:
                    dt = max(stamp_sec - self.prev_imu_stamp_sec, 1e-6)
                gyro_dot = (gyro_in_filter_imu - self.prev_gyro_in_filter_imu) / max(dt, 1e-6)

            translation_effect = np.cross(gyro_dot, self.imu_translation) + np.cross(
                gyro_in_filter_imu, np.cross(gyro_in_filter_imu, self.imu_translation)
            )
            acc_in_filter_imu = acc_in_filter_imu + translation_effect

        self.prev_gyro_in_filter_imu = gyro_in_filter_imu.copy()
        if stamp_sec is not None:
            self.prev_imu_stamp_sec = stamp_sec
        return gyro_in_filter_imu, acc_in_filter_imu

    def get_imu_measurement(self, lowstate_msg: LowState) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        if self.imu_source == "utlidar":
            if self.latest_utlidar_gyro is None or self.latest_utlidar_acc is None:
                return None, None
            gyro = self.latest_utlidar_gyro.copy()
            acc = self.latest_utlidar_acc.copy()
            stamp_sec = self.latest_utlidar_stamp_sec
        else:
            gyro = np.array(lowstate_msg.imu_state.gyroscope, dtype=float)
            acc = np.array(lowstate_msg.imu_state.accelerometer, dtype=float)
            stamp_sec = None

        gyro, acc = self.compensate_imu_transform(gyro, acc, stamp_sec)
        imu_state = np.asarray(np.concatenate([gyro, acc]), dtype=float)
        return imu_state, np.asarray(gyro, dtype=float)

    def feet_transformations(self, state_msg: LowState):
        def unitree_to_urdf_vec(vec):
            return [
                vec[3],
                vec[4],
                vec[5],
                vec[0],
                vec[1],
                vec[2],
                vec[9],
                vec[10],
                vec[11],
                vec[6],
                vec[7],
                vec[8],
            ]

        def feet_contacts(feet_forces):
            contact_threshold = 22
            return [bool(f >= contact_threshold) for f in feet_forces]

        q_unitree = [float(j.q) for j in list(state_msg.motor_state)[:12]]
        v_unitree = [float(j.dq) for j in list(state_msg.motor_state)[:12]]
        f_unitree = list(state_msg.foot_force)

        q_pin = np.array([0] * 6 + [1] + unitree_to_urdf_vec(q_unitree))
        v_pin = np.array([0] * 6 + unitree_to_urdf_vec(v_unitree))
        f_pin = [f_unitree[i] for i in [1, 0, 3, 2]]

        pin.forwardKinematics(self.robot.model, self.robot.data, q_pin, v_pin)
        pin.updateFramePlacements(self.robot.model, self.robot.data)
        pin.computeJointJacobians(self.robot.model, self.robot.data)

        contact_list = feet_contacts(f_pin)
        pose_list = []
        normed_covariance_list = []
        for i in range(4):
            pose_list.append(self.robot.data.oMf[self.foot_frame_id[i]])
            jc = pin.getFrameJacobian(self.robot.model, self.robot.data, self.foot_frame_id[i], pin.LOCAL)[:3, 6:]
            normed_cov_pose = jc @ jc.transpose()
            normed_covariance_list.append(normed_cov_pose)

        return contact_list, pose_list, normed_covariance_list

    def process_lowstate(self, msg: LowState, timestamp_ns: int) -> Optional[OdomSample]:
        imu_state, _gyro = self.get_imu_measurement(msg)
        if imu_state is None:
            return None

        contact_list, pose_list, normed_covariance_list = self.feet_transformations(msg)
        if self.pause:
            if any(contact_list):
                self.pause = False
            else:
                return None

        self.filter.propagate(imu_state, self.dt)

        contact_pairs = []
        kinematics_list = []
        for i in range(len(self.foot_frame_name)):
            contact_pairs.append((i, contact_list[i]))
            velocity = np.zeros(3)
            kinematics = Kinematics(
                i,
                pose_list[i].translation,
                self.joint_pos_noise * normed_covariance_list[i],
                velocity,
                self.contact_vel_noise * np.eye(3),
            )
            kinematics_list.append(kinematics)

        self.filter.setContacts(contact_pairs)
        self.filter.correctKinematics(kinematics_list)
        filter_state = self.filter.getState()

        state_rotation = filter_state.getRotation()
        state_position = filter_state.getPosition().reshape(3)
        state_quaternion = pin.Quaternion(state_rotation)
        state_quaternion.normalize()
        quat_xyzw = np.array([state_quaternion.x, state_quaternion.y, state_quaternion.z, state_quaternion.w])
        return OdomSample(timestamp_ns=timestamp_ns, position=state_position.copy(), quaternion_xyzw=quat_xyzw)


def stamp_to_sec(stamp) -> Optional[float]:
    if stamp is None:
        return None
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


def quaternion_to_euler(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch_y = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.arctan2(t3, t4)
    return roll_x, pitch_y, yaw_z


def wrap_angle(delta: float) -> float:
    return float(np.unwrap(np.array([0.0, delta]))[1])


def resolve_db3_path(path_like: Path) -> Path:
    if path_like.is_file():
        if path_like.suffix != ".db3":
            raise ValueError(f"Expected a .db3 file, got: {path_like}")
        return path_like
    if path_like.is_dir():
        db3_files = sorted(path_like.glob("*.db3"))
        if not db3_files:
            raise ValueError(f"No .db3 files found in directory: {path_like}")
        if len(db3_files) > 1:
            raise ValueError(f"Expected one .db3 file, found {len(db3_files)} in {path_like}")
        return db3_files[0]
    raise ValueError(f"Path does not exist: {path_like}")


def _stats_for(data: Sequence[float]) -> Dict[str, float]:
    arr = np.asarray(data, dtype=float)
    return {
        "mean": float(np.mean(arr)),
        "std": float(np.std(arr)),
        "rmse": float(np.sqrt(np.mean(np.square(arr)))),
        "max_abs": float(np.max(np.abs(arr))),
    }


def synchronize_pairs(
    ref_samples: Sequence[OdomSample],
    est_samples: Sequence[OdomSample],
    max_delta_s: float,
) -> List[Tuple[OdomSample, OdomSample]]:
    if not ref_samples or not est_samples:
        return []
    ref_times = np.array([s.timestamp_ns for s in ref_samples], dtype=np.int64)
    max_delta_ns = int(max_delta_s * 1e9)
    pairs: List[Tuple[OdomSample, OdomSample]] = []
    last_ref_idx = -1

    for est in est_samples:
        start = last_ref_idx + 1
        if start >= len(ref_times):
            break
        idx = int(np.searchsorted(ref_times[start:], est.timestamp_ns)) + start
        candidate_indices = []
        if idx < len(ref_times):
            candidate_indices.append(idx)
        if idx - 1 >= start:
            candidate_indices.append(idx - 1)
        if not candidate_indices:
            continue

        chosen = min(candidate_indices, key=lambda i: abs(int(ref_times[i]) - int(est.timestamp_ns)))
        if abs(int(ref_times[chosen]) - int(est.timestamp_ns)) <= max_delta_ns:
            pairs.append((ref_samples[chosen], est))
            last_ref_idx = chosen
    return pairs


def compute_diff_series(
    synced_pairs: Sequence[Tuple[OdomSample, OdomSample]],
) -> Dict[str, List[float]]:
    out = {
        "time_s": [],
        "pos_diff_x": [],
        "pos_diff_y": [],
        "pos_diff_z": [],
        "orient_diff_roll": [],
        "orient_diff_pitch": [],
        "orient_diff_yaw": [],
    }
    if not synced_pairs:
        return out

    t0_ns = synced_pairs[0][1].timestamp_ns
    for ref, est in synced_pairs:
        out["time_s"].append((est.timestamp_ns - t0_ns) / 1e9)
        pos_delta = ref.position - est.position
        out["pos_diff_x"].append(float(pos_delta[0]))
        out["pos_diff_y"].append(float(pos_delta[1]))
        out["pos_diff_z"].append(float(pos_delta[2]))

        r1, p1, y1 = quaternion_to_euler(
            float(ref.quaternion_xyzw[0]),
            float(ref.quaternion_xyzw[1]),
            float(ref.quaternion_xyzw[2]),
            float(ref.quaternion_xyzw[3]),
        )
        r2, p2, y2 = quaternion_to_euler(
            float(est.quaternion_xyzw[0]),
            float(est.quaternion_xyzw[1]),
            float(est.quaternion_xyzw[2]),
            float(est.quaternion_xyzw[3]),
        )
        out["orient_diff_roll"].append(wrap_angle(r1 - r2))
        out["orient_diff_pitch"].append(wrap_angle(p1 - p2))
        out["orient_diff_yaw"].append(wrap_angle(y1 - y2))
    return out


def print_stats(label: str, synced_pairs: Sequence[Tuple[OdomSample, OdomSample]], series: Dict[str, List[float]]):
    print(f"\n=== {label} ===")
    print(f"Synchronized samples: {len(synced_pairs)}")
    if not synced_pairs:
        print("No synchronized samples found.")
        return

    for axis, key in [("x", "pos_diff_x"), ("y", "pos_diff_y"), ("z", "pos_diff_z")]:
        s = _stats_for(series[key])
        print(
            f"Position {axis} [m] | mean={s['mean']:.6f}, std={s['std']:.6f}, "
            f"rmse={s['rmse']:.6f}, max_abs={s['max_abs']:.6f}"
        )

    for axis, key in [("roll", "orient_diff_roll"), ("pitch", "orient_diff_pitch"), ("yaw", "orient_diff_yaw")]:
        rad_stats = _stats_for(series[key])
        deg_stats = _stats_for(np.rad2deg(series[key]))
        print(
            f"Orientation {axis} | mean={rad_stats['mean']:.6f} rad ({deg_stats['mean']:.3f} deg), "
            f"std={rad_stats['std']:.6f} rad ({deg_stats['std']:.3f} deg), "
            f"rmse={rad_stats['rmse']:.6f} rad ({deg_stats['rmse']:.3f} deg), "
            f"max_abs={rad_stats['max_abs']:.6f} rad ({deg_stats['max_abs']:.3f} deg)"
        )


def save_csv(csv_path: Path, series: Dict[str, List[float]]):
    with csv_path.open("w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "time_s",
                "pos_diff_x_m",
                "pos_diff_y_m",
                "pos_diff_z_m",
                "orient_diff_roll_rad",
                "orient_diff_pitch_rad",
                "orient_diff_yaw_rad",
                "orient_diff_roll_deg",
                "orient_diff_pitch_deg",
                "orient_diff_yaw_deg",
            ]
        )
        for i, t in enumerate(series["time_s"]):
            rr = series["orient_diff_roll"][i]
            pp = series["orient_diff_pitch"][i]
            yy = series["orient_diff_yaw"][i]
            writer.writerow(
                [
                    t,
                    series["pos_diff_x"][i],
                    series["pos_diff_y"][i],
                    series["pos_diff_z"][i],
                    rr,
                    pp,
                    yy,
                    np.rad2deg(rr),
                    np.rad2deg(pp),
                    np.rad2deg(yy),
                ]
            )


def save_plot(plot_path: Path, title_suffix: str, series: Dict[str, List[float]]):
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(2, 1)
    ax[0].plot(series["time_s"], series["pos_diff_x"], label="x diff")
    ax[0].plot(series["time_s"], series["pos_diff_y"], label="y diff")
    ax[0].plot(series["time_s"], series["pos_diff_z"], label="z diff")
    ax[0].legend()
    ax[0].set_title(f"Position Difference (utlidar - {title_suffix})")
    ax[0].set_xlabel("Time (s)")
    ax[0].set_ylabel("Difference (m)")

    ax[1].plot(series["time_s"], np.rad2deg(series["orient_diff_roll"]), label="roll diff")
    ax[1].plot(series["time_s"], np.rad2deg(series["orient_diff_pitch"]), label="pitch diff")
    ax[1].plot(series["time_s"], np.rad2deg(series["orient_diff_yaw"]), label="yaw diff")
    ax[1].legend()
    ax[1].set_title(f"Orientation Difference (utlidar - {title_suffix})")
    ax[1].set_xlabel("Time (s)")
    ax[1].set_ylabel("Difference (deg)")
    fig.tight_layout()
    fig.savefig(str(plot_path))
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Replay a rosbag2 SQLite bag offline, run InEKF odometry twice "
            "(imu_source=lowstate and imu_source=utlidar), and compare both against /utlidar/robot_odom."
        )
    )
    parser.add_argument(
        "--bag",
        type=Path,
        required=True,
        help="Bag directory or .db3 file path.",
    )
    parser.add_argument("--lowstate-topic", default=DEFAULT_LOWSTATE_TOPIC)
    parser.add_argument("--utlidar-imu-topic", default=DEFAULT_UTLIDAR_IMU_TOPIC)
    parser.add_argument("--ref-odom-topic", default=DEFAULT_REF_ODOM_TOPIC)
    parser.add_argument(
        "--sync-window-s",
        type=float,
        default=0.1,
        help="Maximum timestamp difference used for synchronization (default: 0.1s).",
    )
    parser.add_argument("--robot-freq", type=float, default=500.0)
    parser.add_argument("--gyroscope-noise", type=float, default=0.01)
    parser.add_argument("--accelerometer-noise", type=float, default=0.1)
    parser.add_argument("--gyroscope-bias-noise", type=float, default=0.00001)
    parser.add_argument("--accelerometer-bias-noise", type=float, default=0.0001)
    parser.add_argument("--contact-noise", type=float, default=0.001)
    parser.add_argument("--joint-position-noise", type=float, default=0.001)
    parser.add_argument("--contact-velocity-noise", type=float, default=0.001)
    parser.add_argument(
        "--imu-rotation-rpy",
        type=float,
        nargs=3,
        default=(0.0, 0.0, 0.0),
        metavar=("R", "P", "Y"),
    )
    parser.add_argument(
        "--imu-translation-xyz",
        type=float,
        nargs=3,
        default=(0.0, 0.0, 0.0),
        metavar=("X", "Y", "Z"),
    )
    parser.add_argument(
        "--compensate-imu-translation",
        action="store_true",
        help="Enable translation compensation between IMU origins.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("comparison_output"),
        help="Directory where CSV/plots are written.",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Disable plot generation.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    bag_db3 = resolve_db3_path(args.bag)
    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    timestamp_str = datetime.now().strftime("%m%d-%H%M%S")

    filters = {
        "lowstate": OfflineInekf(
            imu_source="lowstate",
            robot_freq=args.robot_freq,
            gyroscope_noise=args.gyroscope_noise,
            accelerometer_noise=args.accelerometer_noise,
            gyroscope_bias_noise=args.gyroscope_bias_noise,
            accelerometer_bias_noise=args.accelerometer_bias_noise,
            contact_noise=args.contact_noise,
            joint_position_noise=args.joint_position_noise,
            contact_velocity_noise=args.contact_velocity_noise,
            imu_rotation_rpy=args.imu_rotation_rpy,
            imu_translation_xyz=args.imu_translation_xyz,
            compensate_imu_translation=args.compensate_imu_translation,
        ),
        "utlidar": OfflineInekf(
            imu_source="utlidar",
            robot_freq=args.robot_freq,
            gyroscope_noise=args.gyroscope_noise,
            accelerometer_noise=args.accelerometer_noise,
            gyroscope_bias_noise=args.gyroscope_bias_noise,
            accelerometer_bias_noise=args.accelerometer_bias_noise,
            contact_noise=args.contact_noise,
            joint_position_noise=args.joint_position_noise,
            contact_velocity_noise=args.contact_velocity_noise,
            imu_rotation_rpy=args.imu_rotation_rpy,
            imu_translation_xyz=args.imu_translation_xyz,
            compensate_imu_translation=args.compensate_imu_translation,
        ),
    }

    estimated_samples: Dict[str, List[OdomSample]] = {"lowstate": [], "utlidar": []}
    ref_samples: List[OdomSample] = []

    topic_to_cls = {
        args.lowstate_topic: LowState,
        args.utlidar_imu_topic: Imu,
        args.ref_odom_topic: Odometry,
    }
    topic_to_type_name = {
        args.lowstate_topic: "unitree_go/msg/LowState",
        args.utlidar_imu_topic: "sensor_msgs/msg/Imu",
        args.ref_odom_topic: "nav_msgs/msg/Odometry",
    }
    for topic, expected_type in topic_to_type_name.items():
        resolved = get_message(expected_type)
        if resolved != topic_to_cls[topic]:
            raise RuntimeError(f"ROS type mismatch for {topic}: expected {expected_type}")

    con = sqlite3.connect(str(bag_db3))
    try:
        topic_rows = con.execute("SELECT id, name, type FROM topics").fetchall()
        id_to_topic: Dict[int, str] = {}
        topic_to_id: Dict[str, int] = {}
        for topic_id, name, type_name in topic_rows:
            id_to_topic[int(topic_id)] = str(name)
            topic_to_id[str(name)] = int(topic_id)
            if name in topic_to_type_name and topic_to_type_name[name] != type_name:
                raise RuntimeError(
                    f"Topic type mismatch for {name}: bag has {type_name}, expected {topic_to_type_name[name]}"
                )

        required_topics = [args.lowstate_topic, args.utlidar_imu_topic, args.ref_odom_topic]
        missing = [t for t in required_topics if t not in topic_to_id]
        if missing:
            raise RuntimeError(f"Missing required topic(s) in bag: {', '.join(missing)}")

        selected_ids = [topic_to_id[t] for t in required_topics]
        placeholders = ",".join("?" for _ in selected_ids)
        query = (
            f"SELECT topic_id, timestamp, data FROM messages "
            f"WHERE topic_id IN ({placeholders}) ORDER BY timestamp ASC"
        )

        for topic_id, timestamp_ns, data in con.execute(query, selected_ids):
            topic = id_to_topic[int(topic_id)]
            msg = deserialize_message(data, topic_to_cls[topic])

            if topic == args.utlidar_imu_topic:
                filters["utlidar"].utlidar_callback(msg)
                continue

            if topic == args.ref_odom_topic:
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                ref_samples.append(
                    OdomSample(
                        timestamp_ns=int(timestamp_ns),
                        position=np.array([float(p.x), float(p.y), float(p.z)]),
                        quaternion_xyzw=np.array([float(q.x), float(q.y), float(q.z), float(q.w)]),
                    )
                )
                continue

            if topic == args.lowstate_topic:
                for key, odom_filter in filters.items():
                    sample = odom_filter.process_lowstate(msg, int(timestamp_ns))
                    if sample is not None:
                        estimated_samples[key].append(sample)
    finally:
        con.close()

    print(f"Bag file: {bag_db3}")
    print(f"Reference topic samples ({args.ref_odom_topic}): {len(ref_samples)}")
    print(f"Estimated samples (imu_source=lowstate): {len(estimated_samples['lowstate'])}")
    print(f"Estimated samples (imu_source=utlidar): {len(estimated_samples['utlidar'])}")

    for key in ["lowstate", "utlidar"]:
        synced_pairs = synchronize_pairs(ref_samples, estimated_samples[key], args.sync_window_s)
        series = compute_diff_series(synced_pairs)
        print_stats(f"Comparison: utlidar/robot_odom - inekf({key} imu)", synced_pairs, series)

        csv_path = output_dir / f"pose_comparison_{key}_{timestamp_str}.csv"
        save_csv(csv_path, series)
        print(f"Saved CSV: {csv_path}")

        if not args.no_plot and synced_pairs:
            plot_path = output_dir / f"pose_comparison_{key}_{timestamp_str}.png"
            save_plot(plot_path, f"inekf({key} imu)", series)
            print(f"Saved plot: {plot_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
