"""
YOLOv3 helper functions — pre-processing, decoding, NMS, drawing, and setup.
Students don't need to read or edit this file.
"""
import os
import time
import random
import colorsys

import numpy as np
import cv2
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from IPython.display import display, Image


# ---------------------------------------------------------------------------
# Class names & colors
# ---------------------------------------------------------------------------

def load_classes(classes_path="img/voc_classes.txt"):
    with open(classes_path) as f:
        class_names = [c.strip() for c in f.readlines()]
    return class_names


def make_colors(class_names):
    n = len(class_names)
    hsv_tuples = [(1.0 * x / n, 1., 1.) for x in range(n)]
    colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
    colors = list(map(lambda x: (int(x[0]*255), int(x[1]*255), int(x[2]*255)), colors))
    random.seed(0)
    random.shuffle(colors)
    random.seed(None)
    return colors


# ---------------------------------------------------------------------------
# Anchor boxes (YOLOv3 VOC defaults)
# ---------------------------------------------------------------------------

def load_anchors():
    anchor_list = [10,13,16,30,33,23,30,61,62,45,59,119,116,90,156,198,373,326]
    return np.array([float(x) for x in anchor_list]).reshape(-1, 2)


# ---------------------------------------------------------------------------
# DPU tensor setup
# ---------------------------------------------------------------------------

def setup_dpu_buffers(overlay):
    """Return (dpu, input_data, output_data, image_buf, shape_in, shape_outs)."""
    dpu = overlay.runner
    inputTensors  = dpu.get_input_tensors()
    outputTensors = dpu.get_output_tensors()

    shapeIn   = tuple(inputTensors[0].dims)
    shapeOut0 = tuple(outputTensors[0].dims)
    shapeOut1 = tuple(outputTensors[1].dims)
    shapeOut2 = tuple(outputTensors[2].dims)

    input_data  = [np.empty(shapeIn,   dtype=np.float32, order="C")]
    output_data = [np.empty(shapeOut0, dtype=np.float32, order="C"),
                   np.empty(shapeOut1, dtype=np.float32, order="C"),
                   np.empty(shapeOut2, dtype=np.float32, order="C")]

    return dpu, input_data, output_data, input_data[0], shapeIn, (shapeOut0, shapeOut1, shapeOut2)


def list_images(image_folder="img"):
    images = [i for i in os.listdir(image_folder) if i.endswith("JPEG")]
    return image_folder, images


# ---------------------------------------------------------------------------
# Pre-processing
# ---------------------------------------------------------------------------

def letterbox_image(image, size):
    ih, iw, _ = image.shape
    w, h = size
    scale = min(w/iw, h/ih)
    nw, nh = int(iw*scale), int(ih*scale)
    image = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_LINEAR)
    new_image = np.ones((h, w, 3), np.uint8) * 128
    h_start, w_start = (h-nh)//2, (w-nw)//2
    new_image[h_start:h_start+nh, w_start:w_start+nw, :] = image
    return new_image


def pre_process(image, model_image_size):
    image = image[...,::-1]
    image_h, image_w, _ = image.shape
    if model_image_size != (None, None):
        assert model_image_size[0] % 32 == 0, 'Multiples of 32 required'
        assert model_image_size[1] % 32 == 0, 'Multiples of 32 required'
        boxed_image = letterbox_image(image, tuple(reversed(model_image_size)))
    else:
        new_image_size = (image_w - (image_w % 32), image_h - (image_h % 32))
        boxed_image = letterbox_image(image, new_image_size)
    image_data = np.array(boxed_image, dtype='float32') / 255.
    return np.expand_dims(image_data, 0)


# ---------------------------------------------------------------------------
# Decoding / NMS
# ---------------------------------------------------------------------------

def _get_feats(feats, anchors, num_classes, input_shape):
    num_anchors = len(anchors)
    anchors_tensor = np.reshape(np.array(anchors, dtype=np.float32), [1,1,1,num_anchors,2])
    grid_size = np.shape(feats)[1:3]
    nu = num_classes + 5
    predictions = np.reshape(feats, [-1, grid_size[0], grid_size[1], num_anchors, nu])
    grid_y = np.tile(np.reshape(np.arange(grid_size[0]), [-1,1,1,1]), [1,grid_size[1],1,1])
    grid_x = np.tile(np.reshape(np.arange(grid_size[1]), [1,-1,1,1]), [grid_size[0],1,1,1])
    grid = np.array(np.concatenate([grid_x, grid_y], axis=-1), dtype=np.float32)
    box_xy = (1/(1+np.exp(-predictions[...,:2])) + grid) / np.array(grid_size[::-1], dtype=np.float32)
    box_wh = np.exp(predictions[...,2:4]) * anchors_tensor / np.array(input_shape[::-1], dtype=np.float32)
    box_confidence  = 1/(1+np.exp(-predictions[...,4:5]))
    box_class_probs = 1/(1+np.exp(-predictions[...,5:]))
    return box_xy, box_wh, box_confidence, box_class_probs


def correct_boxes(box_xy, box_wh, input_shape, image_shape):
    box_yx = box_xy[...,::-1]
    box_hw = box_wh[...,::-1]
    input_shape = np.array(input_shape, dtype=np.float32)
    image_shape = np.array(image_shape, dtype=np.float32)
    new_shape = np.around(image_shape * np.min(input_shape/image_shape))
    offset = (input_shape - new_shape) / 2. / input_shape
    scale  = input_shape / new_shape
    box_yx = (box_yx - offset) * scale
    box_hw *= scale
    box_mins  = box_yx - (box_hw/2.)
    box_maxes = box_yx + (box_hw/2.)
    boxes = np.concatenate([box_mins[...,0:1], box_mins[...,1:2],
                            box_maxes[...,0:1], box_maxes[...,1:2]], axis=-1)
    boxes *= np.concatenate([image_shape, image_shape], axis=-1)
    return boxes


def boxes_and_scores(feats, anchors, classes_num, input_shape, image_shape):
    box_xy, box_wh, box_confidence, box_class_probs = _get_feats(feats, anchors, classes_num, input_shape)
    boxes = np.reshape(correct_boxes(box_xy, box_wh, input_shape, image_shape), [-1, 4])
    box_scores = np.reshape(box_confidence * box_class_probs, [-1, classes_num])
    return boxes, box_scores


def nms_boxes(boxes, scores):
    x1, y1, x2, y2 = boxes[:,0], boxes[:,1], boxes[:,2], boxes[:,3]
    areas = (x2-x1+1)*(y2-y1+1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        w1 = np.maximum(0.0, xx2-xx1+1)
        h1 = np.maximum(0.0, yy2-yy1+1)
        inter = w1*h1
        ovr   = inter / (areas[i] + areas[order[1:]] - inter)
        order = order[np.where(ovr <= 0.55)[0]+1]
    return keep


def evaluate(yolo_outputs, image_shape, class_names, anchors, score_thresh=0.2):
    anchor_mask = [[6,7,8],[3,4,5],[0,1,2]]
    input_shape = np.array(np.shape(yolo_outputs[0])[1:3]) * 32
    boxes, box_scores = [], []
    for i in range(len(yolo_outputs)):
        b, s = boxes_and_scores(yolo_outputs[i], anchors[anchor_mask[i]],
                                len(class_names), input_shape, image_shape)
        boxes.append(b); box_scores.append(s)
    boxes      = np.concatenate(boxes,      axis=0)
    box_scores = np.concatenate(box_scores, axis=0)
    mask = box_scores >= score_thresh
    boxes_, scores_, classes_ = [], [], []
    for c in range(len(class_names)):
        cb = boxes[mask[:,c]]
        cs = box_scores[:,c][mask[:,c]]
        if len(cs) == 0:
            continue
        idx = nms_boxes(cb, cs)
        boxes_.append(cb[idx])
        scores_.append(cs[idx])
        classes_.append(np.ones_like(cs[idx], dtype=np.int32) * c)
    if not boxes_:
        return np.array([]), np.array([]), np.array([], dtype=np.int32)
    return (np.concatenate(boxes_),
            np.concatenate(scores_),
            np.concatenate(classes_))


# ---------------------------------------------------------------------------
# Run — full inference pipeline
# ---------------------------------------------------------------------------

def run(frame, dpu, input_data, output_data, image_buf, shapeIn,
        shapeOut0, shapeOut1, shapeOut2,
        class_names, anchors, colors, display=False):
    """Pre-process frame → DPU → decode → (optionally draw) → return boxes, scores, classes."""
    image_size = frame.shape[:2]
    image_data = np.array(pre_process(frame, (416, 416)), dtype=np.float32)
    image_buf[0, ...] = image_data.reshape(shapeIn[1:])
    job_id = dpu.execute_async(input_data, output_data)
    dpu.wait(job_id)
    yolo_outputs = [
        np.reshape(output_data[0], shapeOut0),
        np.reshape(output_data[1], shapeOut1),
        np.reshape(output_data[2], shapeOut2),
    ]
    boxes, scores, classes = evaluate(yolo_outputs, image_size, class_names, anchors)
    if display:
        draw_boxes(frame, boxes, scores, classes, class_names, colors)
    return boxes, scores, classes


# ---------------------------------------------------------------------------
# Drawing
# ---------------------------------------------------------------------------

def draw_boxes(image, boxes, scores, classes, class_names, colors):
    _, ax = plt.subplots(1)
    ax.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    for i, bbox in enumerate(boxes):
        top, left, bottom, right = bbox
        width, height = right-left, bottom-top
        score, class_index = scores[i], classes[i]
        label = '{}: {:.4f}'.format(class_names[class_index], score)
        color = tuple([c/255 for c in colors[class_index]])
        ax.add_patch(Rectangle((left,top), width, height,
                                edgecolor=color, facecolor='none'))
        ax.annotate(label, (left+width*0.5, top+height*0.5),
                    color=color, weight='bold', fontsize=12,
                    ha='center', va='center')
    return ax
