"""
Confidence Threshold Explorer widget — drag a slider to filter detections.
Students don't need to read or edit this file.
"""
import numpy as np
import cv2
import matplotlib.pyplot as plt
import ipywidgets as widgets
from IPython.display import display as ipy_display

from yolo_helpers import boxes_and_scores, nms_boxes


def launch_confidence_explorer(frame, dpu_state, class_names, colors, anchors):
    """
    Run inference once at a very low threshold, then let students drag a slider
    to see how confidence cutoff affects what gets drawn — no re-inference needed.

    dpu_state must be the dict returned by setup_dpu_buffers(), i.e.:
        {'dpu': dpu, 'input_data': input_data, 'output_data': output_data,
         'image': image_buf, 'shapeIn': shapeIn, 'shapeOuts': (s0,s1,s2)}
    """
    from yolo_helpers import pre_process

    dpu        = dpu_state['dpu']
    input_data = dpu_state['input_data']
    output_data= dpu_state['output_data']
    image_buf  = dpu_state['image']
    shapeIn    = dpu_state['shapeIn']
    shapeOut0, shapeOut1, shapeOut2 = dpu_state['shapeOuts']

    # --- Run inference once at a very low threshold ---
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
    anchor_mask = [[6,7,8],[3,4,5],[0,1,2]]
    input_shape = np.array(np.shape(yolo_outputs[0])[1:3]) * 32
    all_boxes, all_scores = [], []
    for i in range(3):
        b, s = boxes_and_scores(yolo_outputs[i], anchors[anchor_mask[i]],
                                len(class_names), input_shape, image_size)
        all_boxes.append(b); all_scores.append(s)
    raw_boxes  = np.concatenate(all_boxes)
    raw_scores = np.concatenate(all_scores)
    src_frame  = frame.copy()

    def draw_at_threshold(threshold):
        _, ax = plt.subplots(1, figsize=(10, 6))
        ax.imshow(cv2.cvtColor(src_frame, cv2.COLOR_BGR2RGB))
        count = 0
        mask = raw_scores >= threshold
        for c in range(len(class_names)):
            cb = raw_boxes[mask[:,c]]
            cs = raw_scores[:,c][mask[:,c]]
            if len(cs) == 0:
                continue
            for idx in nms_boxes(cb, cs):
                count += 1
                top, left, bottom, right = cb[idx]
                w, h = right-left, bottom-top
                color = tuple([v/255 for v in colors[c]])
                ax.add_patch(plt.Rectangle((left, top), w, h,
                                           edgecolor=color, facecolor='none', linewidth=2))
                ax.annotate('{}: {:.0f}%'.format(class_names[c], cs[idx]*100),
                            (left+w*0.5, top+h*0.5), color=color,
                            weight='bold', fontsize=10, ha='center', va='center')
        ax.set_title('Threshold: {:.2f}  |  Boxes shown: {}'.format(threshold, count), fontsize=13)
        plt.axis('off')
        plt.tight_layout()
        plt.show()

    thresh_slider = widgets.FloatSlider(
        value=0.2, min=0.05, max=0.95, step=0.05,
        description='Min confidence:',
        style={'description_width': 'initial'},
        layout=widgets.Layout(width='500px'),
        continuous_update=False
    )
    thresh_output = widgets.Output()

    def on_change(change):
        thresh_output.clear_output(wait=True)
        with thresh_output:
            draw_at_threshold(change['new'])

    thresh_slider.observe(on_change, names='value')
    ipy_display(thresh_slider, thresh_output)
    with thresh_output:
        draw_at_threshold(0.2)

    print("Done — drag the slider to explore!")
