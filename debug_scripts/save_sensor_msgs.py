#!/usr/bin/env python3
import argparse
import signal
import shlex
import shutil
import subprocess
import sys
from datetime import datetime
from pathlib import Path


DEFAULT_DURATION_SECONDS = 10.0
DEFAULT_TOPICS = ["/lowstate", "/utlidar/imu"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Record raw sensor topics to a ROS 2 bag for offline odometry computation."
        )
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION_SECONDS,
        help=f"Recording length in seconds (default: {DEFAULT_DURATION_SECONDS:g}).",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(f"sensor_msgs_{datetime.now().strftime('%Y%m%d_%H%M%S')}"),
        help=(
            "Output bag directory path or prefix for rosbag2 "
            "(default: sensor_msgs_YYYYMMDD_HHMMSS)."
        ),
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if args.duration <= 0.0:
        raise ValueError("--duration must be > 0 seconds.")

    # ros2 bag record creates a directory; parent must exist.
    parent = args.output.parent
    if not parent.exists():
        raise ValueError(f"Output parent directory does not exist: {parent}")
    if not parent.is_dir():
        raise ValueError(f"Output parent path is not a directory: {parent}")


def run_rosbag_record(duration: float, output: Path) -> int:
    if shutil.which("ros2") is None:
        print("Error: 'ros2' command not found. Source your ROS 2 environment first.", file=sys.stderr)
        return 1

    command = [
        "ros2",
        "bag",
        "record",
        "-o",
        str(output),
        *DEFAULT_TOPICS,
    ]
    print("Recording topics to rosbag2:", flush=True)
    print(f"  topics: {', '.join(DEFAULT_TOPICS)}", flush=True)
    print(f"  duration: {duration:g} s", flush=True)
    print(f"  output: {output}", flush=True)
    print("Running command:", flush=True)
    print(f"  {' '.join(shlex.quote(token) for token in command)}", flush=True)

    try:
        process = subprocess.Popen(command)
    except OSError as error:
        print(f"Error launching ros2 bag record: {error}", file=sys.stderr)
        return 1

    try:
        process.wait(timeout=duration)
        return process.returncode if process.returncode is not None else 0
    except subprocess.TimeoutExpired:
        # Most ROS 2 versions finalize bag metadata correctly on SIGINT.
        print(f"Reached {duration:g} s, stopping rosbag recording...", flush=True)
        process.send_signal(signal.SIGINT)
        try:
            return_code = process.wait(timeout=10.0)
            # Some ROS 2 versions exit with code 2 after SIGINT even when successful.
            return 0 if return_code == 2 else return_code
        except subprocess.TimeoutExpired:
            print("Recorder did not stop after SIGINT, terminating...", file=sys.stderr)
            process.terminate()
            try:
                return process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                print("Recorder still running, killing process.", file=sys.stderr)
                process.kill()
                return process.wait()
    except KeyboardInterrupt:
        print("\nInterrupted by user, stopping rosbag recording...", flush=True)
        process.send_signal(signal.SIGINT)
        try:
            return_code = process.wait(timeout=10.0)
            return 0 if return_code == 2 else return_code
        except subprocess.TimeoutExpired:
            print("Recorder did not stop after SIGINT, terminating...", file=sys.stderr)
            process.terminate()
            try:
                return process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                print("Recorder still running, killing process.", file=sys.stderr)
            process.kill()
            return process.wait()


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)
    except ValueError as error:
        print(f"Argument error: {error}", file=sys.stderr)
        return 2

    return_code = run_rosbag_record(duration=args.duration, output=args.output)
    if return_code == 0:
        print("Bag recording completed successfully.")
    else:
        print(f"ros2 bag record exited with code {return_code}.", file=sys.stderr)
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
