import json

def md(text):
    return {"cell_type": "markdown", "metadata": {}, "source": text.splitlines(keepends=True)}

def code(text):
    return {"cell_type": "code", "metadata": {}, "execution_count": None,
            "outputs": [], "source": text.splitlines(keepends=True)}

cells = []

cells.append(md(
"""# PYNQ Bootcamp AI Helper

Welcome! This notebook opens a friendly AI chat helper for your Kria project.

**How to use it:** click each cell below and press Run (or Shift+Enter), top to bottom. The chat panel will appear right here in the notebook. You can also upload a photo of your project and ask the helper about it.

*Organizers: settings live in bootcamp_ai.py (server IP, model names, cooldown).*"""))

cells.append(md(
"""## Step 1 - One-time setup (run once per board)

This installs the chat interface. It's pinned to a version that works on this Kria image, so it won't break. If it's already installed, this is quick."""))

cells.append(code(
"""# Installs the nice chat UI (pinned for this Kria's Python 3.10 image).
# Safe to re-run; skips if already installed.
import importlib, subprocess, sys

def _ensure(pkg, pip_name=None):
    try:
        importlib.import_module(pkg)
        print("OK -", pkg, "already installed")
    except ImportError:
        print("Installing", pip_name or pkg, "...")
        subprocess.run([sys.executable, "-m", "pip", "install", "-q", pip_name or pkg], check=True)
        print("OK - installed", pip_name or pkg)

_ensure("gradio", "gradio==4.44.1")
print("Setup done - go to Step 2.")"""))

cells.append(md(
"""## Step 2 - Start the helper

Run this cell. A chat panel appears below. Type a question, or click the attach button to add a photo of your project. Use the 'Open in a new tab' button for a full-screen view (great for projectors)."""))

cells.append(code(
"""from bootcamp_ai import launch_chat
launch_chat()"""))

cells.append(md(
"""---
### For the curious

You can also talk to the helper with code instead of the chat box:

```python
from bootcamp_ai import prompt, reset, list_models, use_model

print(prompt("What is a for loop in Python?"))   # ask a question
use_model("Code Helper")                          # switch helpers
print(list_models())                              # see what's on the server
reset()                                           # clear the conversation
```

**Troubleshooting**
- "Can't reach the AI server" -> ask an organizer to check the Strix server is running.
- Chat panel didn't appear -> re-run Step 2. If a kernel was restarted, the server stops until you re-run it.
- Photo not answered -> the helper switches to the vision model automatically; give it a few seconds."""))

nb = {
    "cells": cells,
    "metadata": {
        "kernelspec": {"display_name": "Python 3", "language": "python", "name": "python3"},
        "language_info": {"name": "python", "version": "3.10.12"},
    },
    "nbformat": 4,
    "nbformat_minor": 5,
}

with open("PYNQ_Bootcamp_AI_Helper.ipynb", "w", encoding="utf-8") as f:
    json.dump(nb, f, indent=1, ensure_ascii=False)
print("notebook written")
