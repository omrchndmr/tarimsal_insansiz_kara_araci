# Model Weights

This directory is a placeholder for YOLO model weight files.

## Required Files

| File         | Description                                       | Size   |
|--------------|---------------------------------------------------|--------|
| `best.pt`    | Custom-trained YOLOv8 model for crop row detection | ~5.5 MB |

## How to Obtain

The model weights are **not included** in this repository due to file size.

**Option 1 — Train your own model:**
```bash
pip install ultralytics
yolo train data=your_dataset.yaml model=yolov8n.pt epochs=100
```

**Option 2 — Download the pre-trained base model:**
```bash
# YOLOv8n base model (not fine-tuned for crops)
pip install ultralytics
python -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
```

## Configuring the Model Path

Set the `YOLO_MODEL_PATH` environment variable or the `~model_path` ROS
parameter to point to your model file:

```bash
# Via environment variable
export YOLO_MODEL_PATH=/path/to/best.pt

# Via ROS launch argument
rosrun otonom otonom.py _model_path:=/path/to/best.pt
```
