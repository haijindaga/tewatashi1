#!/usr/bin/env python3
"""Turn the easy_handeye2 result into the URDF camera mount arguments.

  python3 tools/handeye_to_urdf.py [name]      (default name: tewatashi_eih)

The calibration is tool0 -> calib_cam_link, i.e. flange -> RealSense camera_link,
which is exactly camera_mount_xyz / camera_mount_rpy of tewatashi_cell_macro.xacro
(URDF rpy: fixed-axis roll-pitch-yaw, R = Rz(yaw) Ry(pitch) Rx(roll)).
"""
import math
import pathlib
import sys

import yaml


def quat_to_rpy(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "tewatashi_eih"
    path = pathlib.Path.home() / ".ros2/easy_handeye2/calibrations" / f"{name}.calib"
    c = yaml.safe_load(path.read_text())
    p = c["parameters"]
    if p["robot_effector_frame"] != "tool0" or not p["tracking_base_frame"].endswith("_link"):
        raise SystemExit(f"unexpected frames: {p['robot_effector_frame']} -> {p['tracking_base_frame']}")
    t, q = c["transform"]["translation"], c["transform"]["rotation"]
    r, pi, y = quat_to_rpy(q["x"], q["y"], q["z"], q["w"])
    print(f"# from {path}")
    print(f"camera_mount_xyz:='{t['x']:.4f} {t['y']:.4f} {t['z']:.4f}'")
    print(f"camera_mount_rpy:='{r:.4f} {pi:.4f} {y:.4f}'")
    print(f"# distance flange->camera {math.dist((0, 0, 0), (t['x'], t['y'], t['z'])) * 1000:.0f} mm "
          "(check against a ruler: a big mismatch means a bad calibration)")


if __name__ == "__main__":
    main()
