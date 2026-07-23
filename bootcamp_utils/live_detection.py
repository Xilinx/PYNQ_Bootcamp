"""
Live real-time object detection loop.

Concepts covered:
  - Opening a webcam with OpenCV (cv2.VideoCapture)
  - Drawing bounding boxes and labels onto frames (cv2.rectangle, cv2.putText)
  - Streaming annotated frames back to the Jupyter notebook
  - Updating an OLED display with the top detection result
  - Running a blocking loop on a background thread so the notebook UI stays responsive

Students don't need to edit this file — read it to understand the pattern,
then apply the same ideas in the Challenges section.
"""
import threading
import cv2
import ipywidgets as widgets
from IPython.display import display as ipy_display, Image


def launch_live_detection(run, class_names, colors, oled=None):
    """
    Start the live webcam detection loop.

    Parameters
    ----------
    run         : the run() function from the notebook
    class_names : list of class name strings
    colors      : list of (R, G, B) tuples, one per class
    oled        : optional OLED display object (pass None to skip OLED output)
    """
    # ── Controls ───────────────────────────────────────────────────────────────
    stop_button = widgets.ToggleButton(
        value=False,
        description='Stop Camera',
        button_style='danger',
        icon='stop',
        layout=widgets.Layout(width='150px', height='40px')
    )
    thresh_slider = widgets.FloatSlider(
        value=0.3, min=0.05, max=0.95, step=0.05,
        description='Min confidence:',
        style={'description_width': 'initial'},
        layout=widgets.Layout(width='400px'),
        continuous_update=False
    )
    session_classes_seen = set()
    session_label = widgets.Label(value='Objects seen this session: none yet')

    def _loop(stop_btn):
        # ── 1. Open the webcam ────────────────────────────────────────────────
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)       # discard buffered frames
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        display_handle = ipy_display(None, display_id=True)
        last_oled_text = ''

        while not stop_btn.value:
            ret, live_frame = cap.read()           # grab one frame from camera
            if not ret:
                break

            threshold = thresh_slider.value
            boxes, scores, classes = run(live_frame)

            # ── 2. Draw boxes with OpenCV ─────────────────────────────────────
            for i, bbox in enumerate(boxes):
                if scores[i] < threshold:
                    continue
                y_min, x_min, y_max, x_max = map(int, bbox)
                class_idx = classes[i]
                color_bgr = colors[class_idx]      # BGR tuple for OpenCV
                label = '{}: {:.0f}%'.format(class_names[class_idx], scores[i] * 100)
                # Draw bounding rectangle directly onto the frame array
                cv2.rectangle(live_frame, (x_min, y_min), (x_max, y_max), color_bgr, 2)
                # Overlay the class label above the box
                cv2.putText(live_frame, label, (x_min, max(y_min - 8, 15)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2, cv2.LINE_AA)
                session_classes_seen.add(class_names[class_idx])

            seen_str = ', '.join(sorted(session_classes_seen)) if session_classes_seen else 'none yet'
            session_label.value = 'Objects seen this session: ' + seen_str

            # ── 3. Update OLED with the highest-confidence detection ──────────
            if oled is not None and len(scores) > 0:
                above = [(scores[i], classes[i]) for i in range(len(scores)) if scores[i] >= threshold]
                if above:
                    top_name = class_names[max(above, key=lambda x: x[0])[1]]
                    if top_name != last_oled_text:  # only write when it changes
                        oled.clear_display()
                        oled.put_string(top_name)
                        last_oled_text = top_name

            # ── 4. Encode annotated frame as JPEG and push to notebook ────────
            _, encoded = cv2.imencode('.jpeg', live_frame)
            display_handle.update(Image(data=encoded.tobytes()))

        # ── 5. Release the camera when the stop button is pressed ─────────────
        cap.release()
        display_handle.update(None)

    # Run the loop on a background thread so the notebook UI stays interactive
    ipy_display(widgets.HBox([stop_button, thresh_slider]))
    ipy_display(session_label)
    threading.Thread(target=_loop, args=(stop_button,)).start()
