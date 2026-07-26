"""
DPU utility functions — reusable across all bootcamp notebooks that use the DPU.
"""
import os
import numpy as np


def setup_dpu_buffers(overlay):
    """
    Wire up DPU input/output memory buffers from a loaded overlay.

    Returns
    -------
    dpu, input_data, output_data, image_buf, shapeIn, shape_outs

    shape_outs is a tuple of output shape tuples, one per output tensor.

    Example
    -------
        dpu, input_data, output_data, image, shapeIn, (shapeOut0,) = setup_dpu_buffers(overlay)
    """
    dpu = overlay.runner
    input_tensors  = dpu.get_input_tensors()
    output_tensors = dpu.get_output_tensors()

    shapeIn     = tuple(input_tensors[0].dims)
    shape_outs  = tuple(tuple(t.dims) for t in output_tensors)

    input_data  = [np.empty(shapeIn, dtype=np.float32, order='C')]
    output_data = [np.empty(s, dtype=np.float32, order='C') for s in shape_outs]

    return dpu, input_data, output_data, input_data[0], shapeIn, shape_outs


def list_images(image_folder, extensions=('JPEG', 'jpg', 'jpeg', 'png')):
    """
    Return a sorted list of image filenames in image_folder matching extensions.

    Returns
    -------
    image_folder, filenames
    """
    exts = tuple(e.lower() for e in extensions)
    files = sorted(
        f for f in os.listdir(image_folder)
        if f.rsplit('.', 1)[-1].lower() in exts
    )
    return image_folder, files
