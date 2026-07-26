"""
Color utilities — reusable across any bootcamp notebook with named classes.
"""
import random
import colorsys


def load_classes(classes_path):
    """Read class names from a text file, one per line."""
    with open(classes_path) as f:
        return [c.strip() for c in f.readlines()]


def make_colors(class_names, seed=0):
    """
    Generate a unique, shuffled RGB color for each class name.

    Returns
    -------
    list of (R, G, B) int tuples
    """
    n = len(class_names)
    hsv_tuples = [(x / n, 1., 1.) for x in range(n)]
    colors = [colorsys.hsv_to_rgb(*h) for h in hsv_tuples]
    colors = [(int(r * 255), int(g * 255), int(b * 255)) for r, g, b in colors]
    random.seed(seed)
    random.shuffle(colors)
    random.seed(None)
    return colors
