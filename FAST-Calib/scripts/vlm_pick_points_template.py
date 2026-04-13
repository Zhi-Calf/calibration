#!/usr/bin/env python3
"""
GroundingDINO + SAM auto-pick script.

Outputs 4 image points (u v) in clockwise order:
u0 v0
u1 v1
u2 v2
u3 v3

Usage example:
python vlm_pick_points_template.py \
  --image /mnt/f/AI/calibration/data/debug_cam0.png \
  --output /mnt/f/AI/calibration/data/vlm_points_cam0.txt \
  --dino-config /path/to/GroundingDINO_SwinT_OGC.py \
  --dino-checkpoint /path/to/groundingdino_swint_ogc.pth \
  --sam-checkpoint /path/to/sam_vit_h_4b8939.pth
"""

import argparse
from pathlib import Path

import cv2
import numpy as np


def order_points_clockwise(pts: np.ndarray) -> np.ndarray:
    # Start from top-left then clockwise.
    center = np.mean(pts, axis=0)
    angles = np.arctan2(pts[:, 1] - center[1], pts[:, 0] - center[0])
    idx = np.argsort(angles)
    ordered = pts[idx]
    top_left_idx = np.argmin(ordered[:, 0] + ordered[:, 1])
    ordered = np.roll(ordered, -top_left_idx, axis=0)
    return ordered


def corners_from_mask(mask: np.ndarray) -> np.ndarray:
    cnts, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        raise RuntimeError("No contour found from SAM mask.")
    cnt = max(cnts, key=cv2.contourArea)
    rect = cv2.minAreaRect(cnt)
    box = cv2.boxPoints(rect).astype(np.float32)
    return order_points_clockwise(box)


def corners_from_box_xyxy(box_xyxy: np.ndarray) -> np.ndarray:
    x1, y1, x2, y2 = box_xyxy.astype(np.float32).tolist()
    pts = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=np.float32)
    return order_points_clockwise(pts)


def save_points(points: np.ndarray, output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as f:
        for p in points:
            f.write(f"{p[0]:.3f} {p[1]:.3f}\n")


def draw_debug(image_bgr: np.ndarray, corners: np.ndarray, box_xyxy: np.ndarray, output: Path) -> None:
    vis = image_bgr.copy()
    x1, y1, x2, y2 = box_xyxy.astype(int).tolist()
    cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 255), 2)
    for i, p in enumerate(corners):
        cv2.circle(vis, (int(p[0]), int(p[1])), 6, (0, 0, 255), -1)
        cv2.putText(vis, str(i + 1), (int(p[0]) + 8, int(p[1]) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    dbg = output.with_suffix(output.suffix + ".viz.png")
    cv2.imwrite(str(dbg), vis)
    print(f"[vlm] Debug image saved to: {dbg}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True, help="Input image path")
    parser.add_argument("--output", required=True, help="Output file path for 4 points")
    parser.add_argument("--text-prompt", default="calibration board", help="Grounding text prompt")
    parser.add_argument("--box-threshold", type=float, default=0.30)
    parser.add_argument("--text-threshold", type=float, default=0.25)
    parser.add_argument("--device", default="cuda", help="cuda or cpu")
    parser.add_argument("--sam-model-type", default="vit_h", help="vit_h / vit_l / vit_b")
    parser.add_argument("--dino-config", required=True, help="GroundingDINO config .py")
    parser.add_argument("--dino-checkpoint", required=True, help="GroundingDINO checkpoint .pth")
    parser.add_argument("--sam-checkpoint", required=True, help="SAM checkpoint .pth")
    parser.add_argument("--no-sam", action="store_true", help="Use DINO box corners directly")
    args = parser.parse_args()

    image_path = Path(args.image)
    output_path = Path(args.output)
    image_bgr = cv2.imread(str(image_path))
    if image_bgr is None:
        raise RuntimeError(f"Cannot read image: {image_path}")
    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    h, w = image_rgb.shape[:2]

    try:
        import torch
        from groundingdino.util.inference import load_model, predict
    except Exception as e:
        raise RuntimeError(
            "Failed to import GroundingDINO. Install it first."
        ) from e

    model = load_model(args.dino_config, args.dino_checkpoint, device=args.device)
    boxes, logits, _ = predict(
        model=model,
        image=image_rgb,
        caption=args.text_prompt,
        box_threshold=args.box_threshold,
        text_threshold=args.text_threshold,
        device=args.device,
    )
    if boxes is None or len(boxes) == 0:
        raise RuntimeError("GroundingDINO found no box.")

    # groundingdino predict returns normalized cxcywh.
    if hasattr(boxes, "detach"):
        boxes_np = boxes.detach().cpu().numpy()
    else:
        boxes_np = np.asarray(boxes)
    if hasattr(logits, "detach"):
        logits_np = logits.detach().cpu().numpy()
    else:
        logits_np = np.asarray(logits)

    best_idx = int(np.argmax(logits_np))
    cx, cy, bw, bh = boxes_np[best_idx].tolist()
    x1 = (cx - bw / 2.0) * w
    y1 = (cy - bh / 2.0) * h
    x2 = (cx + bw / 2.0) * w
    y2 = (cy + bh / 2.0) * h
    box_xyxy = np.array([x1, y1, x2, y2], dtype=np.float32)
    box_xyxy[0::2] = np.clip(box_xyxy[0::2], 0, w - 1)
    box_xyxy[1::2] = np.clip(box_xyxy[1::2], 0, h - 1)

    if args.no_sam:
        corners = corners_from_box_xyxy(box_xyxy)
    else:
        try:
            from segment_anything import SamPredictor, sam_model_registry
        except Exception as e:
            raise RuntimeError("Failed to import SAM. Install segment-anything.") from e
        sam = sam_model_registry[args.sam_model_type](checkpoint=args.sam_checkpoint)
        sam.to(device=args.device)
        predictor = SamPredictor(sam)
        predictor.set_image(image_rgb)
        masks, scores, _ = predictor.predict(
            box=box_xyxy,
            multimask_output=True,
        )
        if masks is None or len(masks) == 0:
            raise RuntimeError("SAM found no mask.")
        best_mask = masks[int(np.argmax(scores))]
        corners = corners_from_mask(best_mask)

    save_points(corners, output_path)
    draw_debug(image_bgr, corners, box_xyxy, output_path)
    print(f"[vlm] Wrote 4 points to {output_path}")


if __name__ == "__main__":
    main()
