# Genesis Server Installation Guide

Step-by-step installation for different hardware configurations.

## Quick Install (Choose One)

### Option 1: CPU Only (Any Machine)

```bash
cd /home/trabalgi/workspace/dev/bootcamp/PYNQ_Bootcamp_Final/genesis/server

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install PyTorch CPU version
pip install torch --index-url https://download.pytorch.org/whl/cpu

# Install Genesis and dependencies
pip install -e .

# Set backend to CPU
export GENESIS_BACKEND=cpu
```

### Option 2: AMD GPU (ROCm)

```bash
cd /home/trabalgi/workspace/dev/bootcamp/PYNQ_Bootcamp_Final/genesis/server

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install PyTorch with ROCm support (adjust ROCm version as needed)
pip install torch --index-url https://download.pytorch.org/whl/rocm6.0

# Install Genesis and dependencies
pip install -e .

# Set backend to AMD GPU
export GENESIS_BACKEND=amdgpu
```

### Option 3: NVIDIA GPU (CUDA)

```bash
cd /home/trabalgi/workspace/dev/bootcamp/PYNQ_Bootcamp_Final/genesis/server

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install PyTorch with CUDA support (adjust CUDA version as needed)
pip install torch --index-url https://download.pytorch.org/whl/cu121

# Install Genesis and dependencies
pip install -e .

# Set backend to CUDA
export GENESIS_BACKEND=cuda
```

## Detailed Installation Steps

### Step 1: Check Your Hardware

**To check if you have NVIDIA GPU:**
```bash
nvidia-smi
```

**To check if you have AMD GPU:**
```bash
rocm-smi
# or
lspci | grep -i vga
```

### Step 2: Install System Dependencies

```bash
# Update system
sudo apt-get update

# Install Python and development tools
sudo apt-get install -y python3 python3-pip python3-venv

# Install OpenCV dependencies
sudo apt-get install -y libopencv-dev python3-opencv

# Install image processing libraries
sudo apt-get install -y libjpeg-dev libpng-dev
```

### Step 3: Create Virtual Environment

```bash
cd /home/trabalgi/workspace/dev/bootcamp/PYNQ_Bootcamp_Final/genesis/server

python3 -m venv venv
source venv/bin/activate

# Upgrade pip
pip install --upgrade pip
```

### Step 4: Install PyTorch (Choose Your Backend)

#### CPU Backend (Recommended for Testing)

```bash
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
```

#### AMD GPU Backend (ROCm)

First, check your ROCm version:
```bash
rocm-smi --showproductname
```

Then install matching PyTorch:
```bash
# For ROCm 6.0
pip install torch torchvision --index-url https://download.pytorch.org/whl/rocm6.0

# For ROCm 5.7
pip install torch torchvision --index-url https://download.pytorch.org/whl/rocm5.7
```

#### NVIDIA GPU Backend (CUDA)

First, check your CUDA version:
```bash
nvcc --version
# or
nvidia-smi
```

Then install matching PyTorch:
```bash
# For CUDA 12.1
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121

# For CUDA 11.8
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118
```

### Step 5: Install Genesis and Dependencies

```bash
# Install from setup.py (includes all dependencies)
pip install -e .
```

This will install:
- genesis-world (physics engine)
- numpy
- flask (video streaming)
- opencv-python (video encoding)
- pillow (image processing)

### Step 6: Verify Installation

```bash
# Test PyTorch
python -c "import torch; print(f'PyTorch: {torch.__version__}'); print(f'CUDA available: {torch.cuda.is_available()}')"

# Test Genesis
python -c "import genesis as gs; print(f'Genesis: {gs.__version__}')"

# Test Flask
python -c "import flask; print(f'Flask: {flask.__version__}')"

# Test OpenCV
python -c "import cv2; print(f'OpenCV: {cv2.__version__}')"
```

Expected output:
```
PyTorch: 2.x.x
CUDA available: True  (or False for CPU)
Genesis: x.x.x
Flask: 2.x.x
OpenCV: 4.x.x
```

### Step 7: Configure Environment

Create a config file:
```bash
cat > ~/.genesis_server_config << 'EOF'
# Server ports
export GENESIS_PORT=9002
export GENESIS_STREAM_PORT=8080

# Backend: cpu, cuda, amdgpu
export GENESIS_BACKEND=cpu  # Change based on your hardware

# Disable desktop viewer for headless servers
export GENESIS_SHOW_VIEWER=false

# Server limits
export GENESIS_MAX_SESSIONS=30
export GENESIS_SESSION_TIMEOUT=7200

# Admin password
export GENESIS_ADMIN_PASSWORD=bootcamp2024
EOF

# Load configuration
source ~/.genesis_server_config
```

### Step 8: Test Server

```bash
# Activate environment
source venv/bin/activate
source ~/.genesis_server_config

# Run server
python scripts/run_server.py
```

You should see:
```
===================================================
  Genesis Simulation Server
===================================================
  Available IPs:
    - Main API: http://192.168.1.100:9002
    - Stream: http://192.168.1.100:8080
  Backend    : cpu
  Viewer     : disabled
===================================================
```

### Step 9: Test Connection

In another terminal:
```bash
curl -X POST http://localhost:9002 \
  -H "Content-Type: application/json" \
  -d '{"action": "create_env", "params": {"scene": "pick_and_place"}}'
```

Should return:
```json
{"token": "...", "status": "ok"}
```

## Troubleshooting

### Issue: "No module named 'torch'"

**Solution:**
```bash
# Make sure you installed PyTorch BEFORE genesis-world
pip install torch --index-url https://download.pytorch.org/whl/cpu
pip install -e .
```

### Issue: "CUDA error" or "ROCm error"

**Solution:**
```bash
# Fall back to CPU
export GENESIS_BACKEND=cpu
python scripts/run_server.py
```

### Issue: "genesis-world installation fails"

**Solution:**
```bash
# Install dependencies first
pip install torch numpy
pip install genesis-world

# Then install the rest
pip install flask opencv-python pillow
```

### Issue: "ImportError: libGL.so.1"

**Solution:**
```bash
sudo apt-get install -y libgl1-mesa-glx
```

### Issue: Port already in use

**Solution:**
```bash
# Find what's using the port
sudo lsof -i :9002
sudo lsof -i :8080

# Change the port
export GENESIS_PORT=9003
export GENESIS_STREAM_PORT=8081
```

## Backend Performance Comparison

| Backend | Speed | Requirements | Best For |
|---------|-------|--------------|----------|
| **CPU** | Slow | Any machine | Testing, small groups (<10 students) |
| **CUDA** | Fast | NVIDIA GPU + CUDA | Production, large groups (30+ students) |
| **AMD GPU** | Fast | AMD GPU + ROCm | Production, large groups (30+ students) |

## Recommended Setup for Bootcamp

For a class of 30 students:

**Minimum:**
- CPU: 8+ cores
- RAM: 16GB+
- Backend: CPU
- Students: Up to 15 concurrent

**Recommended:**
- CPU: 16+ cores
- RAM: 32GB+
- GPU: NVIDIA RTX 3060 or AMD equivalent
- Backend: cuda/amdgpu
- Students: 30+ concurrent

**Optimal:**
- CPU: 32+ cores
- RAM: 64GB+
- GPU: NVIDIA RTX 4090 or AMD MI210
- Backend: cuda/amdgpu
- Students: 50+ concurrent

## Quick Reference

```bash
# Activate environment
source venv/bin/activate
source ~/.genesis_server_config

# Start server
python scripts/run_server.py

# Check if running
curl http://localhost:9002

# View logs (if using systemd)
journalctl -u genesis-server -f

# Get server IP
hostname -I | awk '{print $1}'
```

## Next Steps

Once installation is complete:
- See [QUICKSTART.md](QUICKSTART.md) for running the server
- See [SETUP.md](SETUP.md) for production deployment
- See [README.md](README.md) for API reference
