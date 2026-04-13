#!/usr/bin/env python3
"""Extract one image from a ROS2 bag topic and save as PNG.

Requires:
  pip install rosbags numpy opencv-python
"""

import argparse
from pathlib import Path

import cv2
import numpy as np


def image_msg_to_bgr(msg):
    h = int(msg.height)
    w = int(msg.width)
    enc = str(msg.encoding).lower()
    step = int(msg.step)
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if h <= 0 or w <= 0:
        raise RuntimeError(f"Invalid image shape: {h}x{w}")

    if enc in ("bgr8", "rgb8"):
        expected = h * step
        if raw.size < expected:
            raise RuntimeError("Image data size smaller than expected for rgb/bgr.")
        img = raw[:expected].reshape((h, step // 3, 3))[:, :w, :]
        if enc == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img

    if enc in ("mono8", "8uc1"):
        expected = h * step
        if raw.size < expected:
            raise RuntimeError("Image data size smaller than expected for mono8.")
        gray = raw[:expected].reshape((h, step))[:, :w]
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    if enc in ("yuv422_yuy2", "yuyv", "yuv422"):
        # 2 bytes per pixel packed YUY2/YUYV.
        expected = h * step
        if raw.size < expected:
            raise RuntimeError("Image data size smaller than expected for yuv422.")
        # step is bytes per row, so width in packed pairs is step/2
        yuv = raw[:expected].reshape((h, step // 2, 2))
        # keep only valid width
        yuv = yuv[:, :w, :]
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_YUY2)

    raise RuntimeError(f"Unsupported encoding: {msg.encoding}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True, help="ROS2 bag directory path")
    parser.add_argument("--topic", required=True, help="Image topic, e.g. /cam0/image_raw")
    parser.add_argument("--output", required=True, help="Output png path")
    args = parser.parse_args()

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)

    try:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore
    except Exception as e:
        raise RuntimeError(
            "rosbags is required. Install: pip3 install rosbags"
        ) from e

    bag_path = Path(args.bag)
    if not bag_path.exists():
        raise RuntimeError(f"Bag path not found: {bag_path}")

    typestore = get_typestore(Stores.ROS2_HUMBLE)

    with AnyReader([bag_path], default_typestore=typestore) as reader:
        connections = [c for c in reader.connections if c.topic == args.topic]
        if not connections:
            raise RuntimeError(f"Topic not found in bag: {args.topic}")

        for conn, _, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, conn.msgtype)
            bgr = image_msg_to_bgr(msg)
            cv2.imwrite(str(out), bgr)
            print(f"[extract] Saved image: {out}")
            return

    raise RuntimeError(f"No message found on topic: {args.topic}")


if __name__ == "__main__":
    main()
