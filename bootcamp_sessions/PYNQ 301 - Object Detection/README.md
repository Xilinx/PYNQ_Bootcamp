# PYNQ 301 - Object Detection

Real-time object detection using YOLOv3 on the KRIA KV260 board.

## Files Structure

```
PYNQ 301 - Object Detection/
├── PYNQ 301 - Object Detection.ipynb  # Main notebook
├── PYNQ-301_Object_Detection.pptx     # Presentation slides
├── bootcamp_utils/                     # Helper utilities (session-specific)
│   ├── color_utils.py                 # Color palette & class loading
│   ├── dpu_utils.py                   # DPU buffer setup & image listing
│   ├── live_detection.py              # Live webcam detection loop
│   └── __init__.py
├── yolo_helpers.py                     # YOLO-specific helpers
├── class_explorer.py                   # Interactive class filtering widget
├── confidence_explorer.py              # Confidence threshold widget
├── img/                                # Sample images & class names
│   └── voc_classes.txt                # VOC dataset class names
└── tf_yolov3_voc.xmodel               # YOLOv3 model weights
```

## Quick Start

1. Open `PYNQ 301 - Object Detection.ipynb`
2. Run cells in order
3. The notebook will auto-import `bootcamp_utils` from this directory

## What's in bootcamp_utils?

**Local utilities specific to this session:**

- `color_utils.py` - Loads class names and generates color palettes
- `dpu_utils.py` - Sets up DPU input/output buffers
- `live_detection.py` - Production-ready live webcam detection with threading

These utilities are only used in this object detection session and are kept local to avoid cluttering the main repository.

## Import Flow

```
Notebook cell 8:
  ├─> from bootcamp_utils.color_utils import load_classes, make_colors
  ├─> from bootcamp_utils.dpu_utils import setup_dpu_buffers, list_images
  ├─> from bootcamp_utils.live_detection import launch_live_detection
  └─> from yolo_helpers import load_anchors, pre_process, evaluate, draw_boxes
```

`yolo_helpers.py` re-exports some bootcamp_utils functions for convenience.
