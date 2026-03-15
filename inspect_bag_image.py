#!/usr/bin/env python3
"""
Inspect first Image message in a ROS 2 bag (encoding, size, optional save).
For debugging bag image formats. See calibration/README.md.

Usage:
    python inspect_bag_image.py [bag_path]
    python inspect_bag_image.py [bag_path] --save   # decode and write /tmp/first_image.jpg
"""
import sys
from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

bag = sys.argv[1] if len(sys.argv) > 1 else "/mnt/d/rosbag2_2026_03_12-16_58_58"
save_one = "--save" in sys.argv
store = get_typestore(Stores.ROS2_HUMBLE)
with AnyReader([Path(bag)], default_typestore=store) as r:
    for c, ts, raw in r.messages():
        if c.msgtype == "sensor_msgs/msg/Image":
            msg = r.deserialize(raw, c.msgtype)
            enc = getattr(msg, "encoding", None)
            h, w = int(msg.height), int(msg.width)
            data = bytes(msg.data)
            n = len(data) // (h * w) if h and w else 0
            print("encoding:", repr(enc), "h:", h, "w:", w, "bytes:", len(data), "channels_inferred:", n)
            if save_one:
                import numpy as np
                import cv2
                enc = str(enc).strip().lower()
                if enc in ("yuv422_yuy2", "yuy2"):
                    # OpenCV YUV2BGR_YUY2 expects 2-channel (H, W, 2)
                    img = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 2)
                    img = cv2.cvtColor(img, cv2.COLOR_YUV2BGR_YUY2)
                else:
                    print("Unsupported encoding for save:", enc)
                    break
                out = "/tmp/first_image.jpg"
                cv2.imwrite(out, img)
                print("Saved:", out)
            break
    else:
        print("No Image message found")
