"""
Live webcam loop — reusable across any bootcamp notebook with a webcam.

Usage
-----
    from bootcamp_utils.live_webcam import launch_live_webcam

    def my_process(frame):
        # do something with the frame, return annotated frame
        return annotated_frame

    launch_live_webcam(my_process, on_stop=lambda: videoIn.release())
"""
import threading
import cv2
import ipywidgets as widgets
from IPython.display import display as ipy_display, Image


def launch_live_webcam(process_frame, width=640, height=480,
                       on_detection=None, on_stop=None):
    """
    Start a live webcam loop in a background thread.

    Parameters
    ----------
    process_frame : callable(frame) -> annotated_frame
        Called on every webcam frame. Must return the (annotated) frame
        as a BGR numpy array.
    width, height : int
        Webcam capture resolution.
    on_detection : callable(frame, result) -> None, optional
        Called after process_frame if it returns a tuple (annotated_frame, result).
        Use this for peripheral updates (OLED, LED stick, etc.).
    on_stop : callable() -> None, optional
        Called once when the stop button is pressed (e.g. to release hardware).
    """
    stop_button = widgets.ToggleButton(
        value=False,
        description='Stop Camera',
        button_style='danger',
        icon='stop',
        layout=widgets.Layout(width='150px', height='40px')
    )

    def _loop(stop_btn):
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        display_handle = ipy_display(None, display_id=True)

        while not stop_btn.value:
            ret, frame = cap.read()
            if not ret:
                break

            result = process_frame(frame)

            if isinstance(result, tuple):
                annotated, detection_result = result
                if on_detection is not None:
                    on_detection(frame, detection_result)
            else:
                annotated = result

            _, encoded = cv2.imencode('.jpeg', annotated)
            display_handle.update(Image(data=encoded.tobytes()))

        cap.release()
        display_handle.update(None)
        if on_stop is not None:
            on_stop()

    ipy_display(stop_button)
    threading.Thread(target=_loop, args=(stop_button,)).start()
