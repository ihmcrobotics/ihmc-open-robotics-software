# yolo26n-seg

Ultralytics YOLO26 nano instance segmentation, pretrained on COCO (80 classes, same order as `old/yolov8n-seg`).

Exported with the classic one-to-many head, so its outputs have the YOLOv8 layout that `YOLOv8Model` decodes (the same as `old/yolov8n-seg`):
`output0` [1, 116, 19320] (cx, cy, w, h, 80 class scores, 32 mask weights) and `output1` [1, 32, 184, 320]
(mask prototypes), at a fixed 1280 x 736 input.

```python
from ultralytics import YOLO  # ultralytics 8.4.33
YOLO("yolo26n-seg.pt").export(format="onnx", imgsz=(736, 1280), opset=11, simplify=True, dynamic=False, end2end=False)
```

`end2end=False` matters: the default YOLO26 export is NMS free, with a [1, 300, 38] output that `YOLOv8Model` does not read.

`class_names.yaml` sets `opencv_layer_fusion: false`: OpenCV 4.10's CUDA backend fails to fuse this net's layers
(`fuseLayers: biasLayerData->outputBlobsWrappers.size() == 1`). Unfused it runs about as fast as `yolov8n-seg` did.
