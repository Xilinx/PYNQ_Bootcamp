#!/bin/bash

# Set the repository URL and destination folder name
REPO_URL="https://github.com/Xilinx/PYNQ_Bootcamp.git"
LOCAL_DIR="/home/root/jupyter_notebooks/PYNQ_Bootcamp"

# Check if the directory exists
if [ -d "$LOCAL_DIR" ]; then
    echo "Removing existing directory: $LOCAL_DIR"
    rm -rf "$LOCAL_DIR"
else
    echo "Directory does not exist: $LOCAL_DIR"
fi

# Clone the repository
echo "Cloning repository: $REPO_URL"
git clone -b remote-board "$REPO_URL" "$LOCAL_DIR"

# Check if the cloning was successful
if [ $? -eq 0 ]; then
    echo "Repository cloned successfully into $LOCAL_DIR."
    git config --global --add safe.directory /home/root/jupyter_notebooks/PYNQ_Bootcamp
else
    echo "Failed to clone repository."
fi