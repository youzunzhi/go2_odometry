#!/usr/bin/env python3
import argparse
import csv
import sys
import sqlite3
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import numpy as np
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from unitree_go.msg import LowState

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))
from inekf_core import (  # noqa: E402
    DEFAULT_ACCELEROMETER_BIAS_NOISE,
    DEFAULT_ACCELEROMETER_NOISE,
    DEFAULT_CONTACT_NOISE,
    DEFAULT_CONTACT_VELOCITY_NOISE,
    DEFAULT_GYROSCOPE_BIAS_NOISE,
    DEFAULT_GYROSCOPE_NOISE,
    DEFAULT_JOINT_POSITION_NOISE,
    DEFAULT_ROBOT_FREQ,
    InekfCore,
    extract_position_quaternion,
)


DEFAULT_LOWSTATE_TOPIC = "/lowstate"
DEFAULT_REF_ODOM_TOPIC = "/utlidar/robot_odom"


@dataclass
class OdomSample:
    timestamp_ns: int
    position: np.ndarray
    quaternion_xyzw: np.ndarray


def _filter_state_to_odom_sample(filter_state, timestamp_ns: int) -> OdomSample:
    """Extract position and orientation from an InEKF filter state."""
    position, quaternion_xyzw = extract_position_quaternion(filter_state)
    return OdomSample(
        timestamp_ns=timestamp_ns,
        position=position,
        quaternion_xyzw=quaternion_xyzw,
    )


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
            "Replay a rosbag2 SQLite bag offline, run InEKF odometry from lowstate, "
            "and compare against /utlidar/robot_odom."
        )
    )
    parser.add_argument(
        "--bag",
        type=Path,
        required=True,
        help="Bag directory or .db3 file path.",
    )
    parser.add_argument("--lowstate-topic", default=DEFAULT_LOWSTATE_TOPIC)
    parser.add_argument("--ref-odom-topic", default=DEFAULT_REF_ODOM_TOPIC)
    parser.add_argument(
        "--sync-window-s",
        type=float,
        default=0.1,
        help="Maximum timestamp difference used for synchronization (default: 0.1s).",
    )
    parser.add_argument("--robot-freq", type=float, default=DEFAULT_ROBOT_FREQ)
    parser.add_argument("--gyroscope-noise", type=float, default=DEFAULT_GYROSCOPE_NOISE)
    parser.add_argument("--accelerometer-noise", type=float, default=DEFAULT_ACCELEROMETER_NOISE)
    parser.add_argument("--gyroscope-bias-noise", type=float, default=DEFAULT_GYROSCOPE_BIAS_NOISE)
    parser.add_argument("--accelerometer-bias-noise", type=float, default=DEFAULT_ACCELEROMETER_BIAS_NOISE)
    parser.add_argument("--contact-noise", type=float, default=DEFAULT_CONTACT_NOISE)
    parser.add_argument("--joint-position-noise", type=float, default=DEFAULT_JOINT_POSITION_NOISE)
    parser.add_argument("--contact-velocity-noise", type=float, default=DEFAULT_CONTACT_VELOCITY_NOISE)
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

    shared_kwargs = dict(
        robot_freq=args.robot_freq,
        gyroscope_noise=args.gyroscope_noise,
        accelerometer_noise=args.accelerometer_noise,
        gyroscope_bias_noise=args.gyroscope_bias_noise,
        accelerometer_bias_noise=args.accelerometer_bias_noise,
        contact_noise=args.contact_noise,
        joint_position_noise=args.joint_position_noise,
        contact_velocity_noise=args.contact_velocity_noise,
    )
    filter_core = InekfCore(**shared_kwargs)

    estimated_samples: List[OdomSample] = []
    ref_samples: List[OdomSample] = []

    topic_to_cls = {
        args.lowstate_topic: LowState,
        args.ref_odom_topic: Odometry,
    }
    topic_to_type_name = {
        args.lowstate_topic: "unitree_go/msg/LowState",
        args.ref_odom_topic: "nav_msgs/msg/Odometry",
    }
    for topic, expected_type in topic_to_type_name.items():
        resolved = get_message(expected_type)
        if resolved != topic_to_cls[topic]:
            raise RuntimeError(f"ROS type mismatch for {topic}: expected {expected_type}")

    con = sqlite3.connect(str(bag_db3))
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

    required_topics = [args.lowstate_topic, args.ref_odom_topic]
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
            result = filter_core.process_lowstate(msg)
            if not isinstance(result, str):
                filter_state, _gyro = result
                estimated_samples.append(_filter_state_to_odom_sample(filter_state, int(timestamp_ns)))

    con.close()

    print(f"Bag file: {bag_db3}")
    print(f"Reference topic samples ({args.ref_odom_topic}): {len(ref_samples)}")
    print(f"Estimated samples (lowstate IMU): {len(estimated_samples)}")

    synced_pairs = synchronize_pairs(ref_samples, estimated_samples, args.sync_window_s)
    series = compute_diff_series(synced_pairs)
    print_stats("Comparison: utlidar/robot_odom - inekf(lowstate imu)", synced_pairs, series)

    csv_path = output_dir / f"pose_comparison_lowstate_{timestamp_str}.csv"
    save_csv(csv_path, series)
    print(f"Saved CSV: {csv_path}")

    if not args.no_plot and synced_pairs:
        plot_path = output_dir / f"pose_comparison_lowstate_{timestamp_str}.png"
        save_plot(plot_path, "inekf(lowstate imu)", series)
        print(f"Saved plot: {plot_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
