#!/usr/bin/env python3
"""Real-time 2D LiDAR viewer with a rolling time window (default: last 2 seconds)."""

import argparse
import signal
import sys
import time
from collections import deque
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from rplidar import RPLidar, RPLidarException


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Real-time 2D LiDAR viewer (rolling window)")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port (default: /dev/ttyUSB0)")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate (A1: usually 115200)")
    parser.add_argument("--min-distance-mm", type=float, default=80.0, help="Minimum distance filter")
    parser.add_argument("--max-distance-mm", type=float, default=6000.0, help="Maximum distance filter")
    parser.add_argument("--window-sec", type=float, default=2.0, help="Keep only scans from the last N seconds")
    parser.add_argument("--draw-every", type=int, default=2, help="Redraw every N scans")
    parser.add_argument("--max-buf-meas", type=int, default=20000, help="RPLidar input buffer threshold")
    parser.add_argument("--collision-mm", type=float, default=350.0, help="Collision alert threshold")
    parser.add_argument("--max-hard-reconnects", type=int, default=80, help="Maximum hard reconnect attempts")
    parser.add_argument("--reconnect-delay-sec", type=float, default=0.2, help="Delay before retry after stream error")
    parser.add_argument("--save-ply", default="", help="Optional output .ply path on exit (z=0 plane)")
    return parser.parse_args()


def scan_to_xy(scan, args: argparse.Namespace) -> np.ndarray:
    if not scan:
        return np.empty((0, 2), dtype=np.float32)

    angles = np.array([m[1] for m in scan], dtype=np.float32)
    distances = np.array([m[2] for m in scan], dtype=np.float32)

    valid = np.isfinite(distances)
    valid &= distances >= args.min_distance_mm
    valid &= distances <= args.max_distance_mm

    if not np.any(valid):
        return np.empty((0, 2), dtype=np.float32)

    theta = np.deg2rad(angles[valid])
    radial = distances[valid]
    x = radial * np.cos(theta)
    y = radial * np.sin(theta)
    return np.column_stack((x, y)).astype(np.float32)


def write_ply(path: Path, points_xy: np.ndarray) -> None:
    with path.open("w", encoding="utf-8") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(points_xy)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")
        for p in points_xy:
            f.write(f"{p[0]:.3f} {p[1]:.3f} 0.000\n")


def reset_stream_state(lidar: RPLidar) -> None:
    lidar.scanning = [False, 0, "normal"]
    lidar.express_trame = 32
    lidar.express_data = False


def force_clear_serial_input(lidar: RPLidar) -> None:
    serial_obj = getattr(lidar, "_serial", None)
    if serial_obj is None:
        return
    try:
        serial_obj.reset_input_buffer()
    except Exception:
        try:
            serial_obj.flushInput()
        except Exception:
            pass


def patch_safe_get_health(lidar: RPLidar) -> None:
    original_get_health = lidar.get_health

    def safe_get_health():
        last = None
        for _ in range(3):
            result = original_get_health()
            if isinstance(result, tuple) and len(result) == 2:
                return result
            last = result
            force_clear_serial_input(lidar)
            time.sleep(0.03)
        raise RPLidarException(f"Invalid get_health response: {last!r}")

    lidar.get_health = safe_get_health


def soft_resync(lidar: RPLidar, delay_sec: float) -> None:
    try:
        lidar.stop()
    except Exception:
        pass
    reset_stream_state(lidar)
    force_clear_serial_input(lidar)
    time.sleep(max(delay_sec, 0.0))


def hard_reconnect(lidar: RPLidar, delay_sec: float) -> None:
    try:
        lidar.stop()
    except Exception:
        pass
    try:
        lidar.stop_motor()
    except Exception:
        pass
    try:
        lidar.disconnect()
    except Exception:
        pass
    reset_stream_state(lidar)
    time.sleep(max(delay_sec, 0.0))
    lidar.connect()
    force_clear_serial_input(lidar)


def prune_old_scans(scans, now_sec: float, window_sec: float) -> None:
    cutoff = now_sec - max(window_sec, 0.1)
    while scans and scans[0][0] < cutoff:
        scans.popleft()


def should_log(counter: int) -> bool:
    return counter <= 5 or counter % 20 == 0


def main() -> int:
    args = parse_args()
    stop = False

    def handle_sigint(_sig, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        lidar = RPLidar(args.port, baudrate=args.baudrate, timeout=1)
    except Exception as e:
        print(f"Failed to open LiDAR on {args.port}: {e}", file=sys.stderr)
        return 2

    patch_safe_get_health(lidar)

    recent_scans = deque()  # item: (timestamp_sec, points_xy)
    latest_min = float("nan")
    latest_max = float("nan")
    soft_errors = 0
    hard_errors = 0
    scan_idx = 0

    plt.ion()
    _, ax = plt.subplots(num="LiDAR 2D Viewer (Last 2s)")

    try:
        while not stop:
            try:
                for scan in lidar.iter_scans(max_buf_meas=args.max_buf_meas):
                    if stop:
                        break

                    now_sec = time.monotonic()

                    d = np.array([m[2] for m in scan], dtype=np.float32)
                    if d.size:
                        valid = d[(d >= args.min_distance_mm) & (d <= args.max_distance_mm)]
                        if valid.size:
                            latest_min = float(np.min(valid))
                            latest_max = float(np.max(valid))

                    points = scan_to_xy(scan, args)
                    if len(points):
                        recent_scans.append((now_sec, points))

                    prune_old_scans(recent_scans, now_sec, args.window_sec)

                    if scan_idx % args.draw_every == 0:
                        ax.clear()
                        lim = args.max_distance_mm

                        if recent_scans:
                            merged = np.concatenate([p for _, p in recent_scans], axis=0)
                            radial = np.sqrt(merged[:, 0] ** 2 + merged[:, 1] ** 2)
                            near_count = int(np.sum(radial <= args.collision_mm))
                            ax.scatter(merged[:, 0], merged[:, 1], c=radial, s=2.0, cmap="turbo", alpha=0.85)
                            ax.set_title(
                                f"window={args.window_sec:.1f}s points={len(merged)} min={latest_min:.1f}mm "
                                f"max={latest_max:.1f}mm near<{args.collision_mm:.0f}mm={near_count} "
                                f"soft_err={soft_errors} hard_reconn={hard_errors}"
                            )
                        else:
                            ax.set_title("Waiting for LiDAR data...")

                        ax.scatter([0.0], [0.0], c="red", s=25, marker="x")
                        danger = plt.Circle(
                            (0.0, 0.0),
                            args.collision_mm,
                            color="red",
                            fill=False,
                            linestyle="--",
                            linewidth=0.9,
                            alpha=0.65,
                        )
                        ax.add_patch(danger)
                        ax.set_xlim(-lim, lim)
                        ax.set_ylim(-lim, lim)
                        ax.set_aspect("equal", adjustable="box")
                        ax.set_xlabel("X (mm)")
                        ax.set_ylabel("Y (mm)")
                        plt.pause(0.001)

                    scan_idx += 1

            except (RPLidarException, OSError, ValueError) as e:
                msg = str(e)
                recoverable_frame_error = (
                    "Check bit not equal to 1" in msg
                    or "New scan flags mismatch" in msg
                    or "Invalid get_health response" in msg
                    or "too many values to unpack" in msg
                )

                if recoverable_frame_error:
                    soft_errors += 1
                    if should_log(soft_errors):
                        print(f"[warn] LiDAR frame error: {msg} (soft-resync {soft_errors})", file=sys.stderr)
                    soft_resync(lidar, args.reconnect_delay_sec)
                    continue

                hard_errors += 1
                print(
                    f"[warn] LiDAR stream I/O error: {msg} "
                    f"(hard reconnect {hard_errors}/{args.max_hard_reconnects})",
                    file=sys.stderr,
                )
                if hard_errors > args.max_hard_reconnects:
                    print("[error] Reconnect limit exceeded. Exiting.", file=sys.stderr)
                    return 3
                hard_reconnect(lidar, args.reconnect_delay_sec)
                continue

    except KeyboardInterrupt:
        pass
    finally:
        try:
            lidar.stop()
        except Exception:
            pass
        try:
            lidar.stop_motor()
        except Exception:
            pass
        try:
            lidar.disconnect()
        except Exception:
            pass

    if args.save_ply and recent_scans:
        merged = np.concatenate([p for _, p in recent_scans], axis=0)
        out = Path(args.save_ply)
        write_ply(out, merged)
        print(f"Saved {len(merged)} points to {out}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
