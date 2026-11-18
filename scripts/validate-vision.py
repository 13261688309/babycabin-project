"""Re-evaluate existing weights on the existing held-out split without saving images."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from companion.runtime import default_weights, file_digest
from companion.state import LABELS


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights", type=Path, default=default_weights())
    parser.add_argument("--data", type=Path)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--output", type=Path, default=Path("build/vision/validation"))
    args = parser.parse_args()
    if args.data is None:
        args.data = args.weights.resolve().parents[4] / "data/baby_state_yolo_clean/data.yaml"
    if not args.weights.is_file() or not args.data.is_file():
        parser.error("local weights/data not found")
    if args.batch < 1:
        parser.error("batch must be positive")
    # Use an absolute dataset root rather than Ultralytics' global datasets directory.
    import yaml
    from ultralytics import YOLO
    import torch
    import ultralytics

    model = YOLO(str(args.weights.resolve()))
    if model.task != "detect" or set(model.names.values()) != LABELS:
        parser.error("unexpected detection model classes")
    config = yaml.safe_load(args.data.read_text(encoding="utf-8"))
    config["path"] = str(args.data.resolve().parent)
    args.output.mkdir(parents=True, exist_ok=False)
    resolved = args.output / "resolved-data.yaml"
    resolved.write_text(yaml.safe_dump(config, allow_unicode=True), encoding="utf-8")
    validation_info = {}
    model.add_callback("on_val_end", lambda validator: validation_info.update(images=int(validator.seen)))
    metrics = model.val(data=str(resolved.resolve()), split="test", imgsz=640,
                        batch=args.batch, device=args.device, workers=0, plots=False,
                        save_json=False, project=str(args.output.resolve()), name="test", verbose=False)
    report = {
        "weightsSha256": file_digest(args.weights),
        "split": "test",
        "images": validation_info["images"],
        "instances": int(metrics.nt_per_class.sum()),
        "runtime": {"ultralytics": ultralytics.__version__, "torch": torch.__version__, "device": args.device},
        "aggregate": {"precision": float(metrics.box.mp), "recall": float(metrics.box.mr),
                      "map50": float(metrics.box.map50), "map50_95": float(metrics.box.map)},
        "perClass": {},
        "speedMsPerImage": metrics.speed,
        "caution": "Offline box-detection metrics, not clinical accuracy, temporal-filter accuracy, or live-camera acceptance.",
    }
    for index, class_id in enumerate(metrics.box.ap_class_index):
        report["perClass"][model.names[int(class_id)]] = {
            "precision": float(metrics.box.p[index]), "recall": float(metrics.box.r[index]),
            "map50": float(metrics.box.ap50[index]), "map50_95": float(metrics.box.ap[index]),
        }
    destination = args.output / "report.json"
    destination.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f"Validation metadata: {destination.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
