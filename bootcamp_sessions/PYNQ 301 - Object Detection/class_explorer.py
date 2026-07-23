"""
Class Explorer widget — filter detections by class using a dropdown.
Students don't need to read or edit this file.
"""
import cv2
import matplotlib.pyplot as plt
import ipywidgets as widgets
from IPython.display import display as ipy_display


def launch_class_explorer(image, boxes, scores, classes, class_names, colors):
    """Display a dropdown that filters bounding boxes by detected class."""

    def draw_filtered_boxes(filter_class_idx):
        _, ax = plt.subplots(1, figsize=(10, 6))
        ax.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        found = False
        for i, bbox in enumerate(boxes):
            if classes[i] != filter_class_idx:
                continue
            found = True
            top, left, bottom, right = bbox
            width, height = right-left, bottom-top
            label = '{}: {:.0f}%'.format(class_names[filter_class_idx], scores[i]*100)
            color = tuple([c/255 for c in colors[filter_class_idx]])
            ax.add_patch(plt.Rectangle((left, top), width, height,
                                       edgecolor=color, facecolor='none', linewidth=2))
            ax.annotate(label, (left+width*0.5, top+height*0.5),
                        color=color, weight='bold', fontsize=12,
                        ha='center', va='center')
        if not found:
            ax.set_title('No "{}" detected.'.format(class_names[filter_class_idx]),
                         fontsize=14, color='red')
        plt.axis('off')
        plt.tight_layout()
        plt.show()

    detected_idxs = sorted(set(classes.tolist())) if len(classes) > 0 else []
    separator_val = detected_idxs[0] if detected_idxs else 0
    dropdown_options = (
        [('✅ ' + class_names[i], i) for i in detected_idxs]
        + [('─────────────', separator_val)]
        + [('   ' + class_names[i], i) for i in range(len(class_names)) if i not in detected_idxs]
    )

    class_dropdown = widgets.Dropdown(
        options=dropdown_options,
        description='Show class:',
        style={'description_width': 'initial'},
        layout=widgets.Layout(width='300px')
    )
    explorer_output = widgets.Output()

    def on_change(change):
        explorer_output.clear_output(wait=True)
        with explorer_output:
            draw_filtered_boxes(change['new'])

    class_dropdown.observe(on_change, names='value')

    print("Detected classes:", [class_names[i] for i in detected_idxs] if detected_idxs else "None")
    ipy_display(class_dropdown, explorer_output)

    if detected_idxs:
        with explorer_output:
            draw_filtered_boxes(detected_idxs[0])
