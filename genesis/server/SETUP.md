# Genesis Server Setup Guide

Complete setup instructions for running the Genesis simulation server with integrated video streaming for the PYNQ Robotics Bootcamp.

## Architecture Overview

The Genesis server runs **two services on one machine**:

1. **Main API Server** (port 9002) - Handles robot control commands via HTTP JSON API
2. **Video Streaming Server** (port 8080) - Provides live MJPEG video streams of simulations

Both run simultaneously in the same Python process, sharing the simulation state.

```
┌─────────────────────────────────────────────┐
│         Server Machine                      │
│                                             │
│  ┌──────────────────────────────────────┐  │
│  │  Main Process (Python)               │  │
│  │                                      │  │
│  │  ┌────────────────────────────────┐ │  │
│  │  │  HTTP API Server (port 9002)   │ │  │
│  │  │  - Robot commands              │ │  │
│  │  │  - Physics simulation          │ │  │
│  │  └────────────────────────────────┘ │  │
│  │                                      │  │
│  │  ┌────────────────────────────────┐ │  │
│  │  │ Flask Stream Server (port 8080)│ │  │
│  │  │  - MJPEG video streaming       │ │  │
│  │  │  - Web viewer UI               │ │  │
│  │  └────────────────────────────────┘ │  │
│  │                                      │  │
│  │  Both share → [Simulations Dict]    │  │
│  └──────────────────────────────────────┘  │
└─────────────────────────────────────────────┘
```

## Prerequisites

### System Requirements

- **OS**: Linux (Ubuntu 20.04+ recommended)
- **GPU**: NVIDIA GPU with CUDA support (recommended) OR AMD GPU with ROCm
- **CPU**: 4+ cores recommended for multiple students
- **RAM**: 8GB minimum, 16GB+ recommended
- **Python**: 3.8 or later

### Software Dependencies

```bash
# Update system
sudo apt-get update
sudo apt-get upgrade

# Install Python and development tools
sudo apt-get install -y python3 python3-pip python3-venv git

# Install system libraries for OpenCV and video processing
sudo apt-get install -y libopencv-dev python3-opencv
sudo apt-get install -y libjpeg-dev libpng-dev
```

### GPU Setup (Optional but Recommended)

**For NVIDIA GPU:**
```bash
# Install CUDA toolkit (adjust version as needed)
# See: https://developer.nvidia.com/cuda-downloads

# Verify CUDA
nvcc --version
nvidia-smi
```

**For AMD GPU:**
```bash
# Install ROCm (if using AMD GPU)
# See: https://rocm.docs.amd.com/en/latest/deploy/linux/quick_start.html

# Verify ROCm
rocm-smi
```

## Installation

### Step 1: Clone or Navigate to Server Directory

```bash
cd /home/trabalgi/workspace/dev/bootcamp/PYNQ_Bootcamp_Final/genesis/server
```

### Step 2: Create Python Virtual Environment

```bash
# Create virtual environment
python3 -m venv venv

# Activate virtual environment
source venv/bin/activate

# Upgrade pip
pip install --upgrade pip
```

### Step 3: Install Python Dependencies

```bash
# Install all required packages
pip install -e .

# Or install from requirements.txt directly
pip install -r requirements.txt
```

This will install:
- `genesis-world` - Physics simulation engine
- `numpy` - Numerical computing
- `flask>=2.0.0` - Web server for video streaming
- `opencv-python` - Video encoding
- `pillow` - Image processing

### Step 4: Verify Installation

```bash
# Test that Genesis imports correctly
python -c "import genesis as gs; print('Genesis version:', gs.__version__)"

# Test that Flask imports
python -c "import flask; print('Flask version:', flask.__version__)"
```

## Configuration

### Environment Variables

Create a configuration file or set environment variables:

```bash
# Create config file (optional)
cat > ~/.genesis_server_config << 'EOF'
# Main API server port
export GENESIS_PORT=9002

# Video streaming server port
export GENESIS_STREAM_PORT=8080

# Backend: cpu, gpu, cuda, amdgpu, metal
export GENESIS_BACKEND=amdgpu

# Show Genesis desktop viewer (set to false for headless servers)
export GENESIS_SHOW_VIEWER=true

# Admin password for administrative actions
export GENESIS_ADMIN_PASSWORD=bootcamp2024

# Maximum concurrent student sessions
export GENESIS_MAX_SESSIONS=30

# Session timeout (2 hours = 7200 seconds)
export GENESIS_SESSION_TIMEOUT=7200
EOF

# Load configuration
source ~/.genesis_server_config
```

### Configuration Reference

| Variable | Default | Description |
|----------|---------|-------------|
| `GENESIS_PORT` | 9002 | Main API server port (students connect here) |
| `GENESIS_STREAM_PORT` | 8080 | Video streaming server port (for browser viewing) |
| `GENESIS_BACKEND` | amdgpu | Compute backend: `cpu`, `gpu`, `cuda`, `amdgpu`, `metal` |
| `GENESIS_SHOW_VIEWER` | true | Show Genesis desktop viewer (disable for headless) |
| `GENESIS_ADMIN_PASSWORD` | admin123 | Admin password for server management |
| `GENESIS_MAX_SESSIONS` | 10 | Maximum concurrent student sessions |
| `GENESIS_SESSION_TIMEOUT` | 7200 | Idle session timeout in seconds (2 hours) |

### Network Configuration

```bash
# Get your server's IP address
ip addr show | grep "inet " | grep -v 127.0.0.1

# Example output: 192.168.1.100

# Allow ports through firewall
sudo ufw allow 9002/tcp   # Main API
sudo ufw allow 8080/tcp   # Video streaming

# For institutional networks, coordinate with IT to:
# - Open ports 9002 and 8080
# - Ensure students' subnet can reach server IP
```

## Running the Server

### Method 1: Using the Script (Recommended)

```bash
# Activate virtual environment
cd /home/trabalgi/workspace/dev/bootcamp/PYNQ_Bootcamp_Final/genesis/server
source venv/bin/activate

# Load configuration (if using config file)
source ~/.genesis_server_config

# Run server
python scripts/run_server.py
```

### Method 2: As a Python Module

```bash
# From the server directory
python -m genesis_server.server
```

### Expected Output

```
===================================================
  Genesis Simulation Server
===================================================
  Available IPs:
    - Main API: http://192.168.1.100:9002
    - Stream: http://192.168.1.100:8080
  Backend    : amdgpu
  Viewer     : disabled
===================================================
  Example: SimulationClient("192.168.1.100", 9002)
===================================================

  Stream Server: http://localhost:8080
===================================================

  Server running. Press Ctrl+C to stop.
```

## Verification & Testing

### Test Main API Server

```bash
# From another terminal, test the API
curl -X POST http://localhost:9002 \
  -H "Content-Type: application/json" \
  -d '{
    "action": "create_env",
    "params": {"scene": "pick_and_place"}
  }'

# Expected response:
# {"token": "abc123...", "status": "ok"}
```

### Test Video Streaming Server

```bash
# Open in browser:
# http://SERVER_IP:8080

# You should see a web interface listing active sessions
```

### Test from Student Perspective

Create a test script `test_student.py`:

```python
from pynqsim import SimulationClient

# Connect to server (replace with your server IP)
sim = SimulationClient("192.168.1.100", 9002)

# Create environment
sim.create_environment(scene="pick_and_place")
print("Environment created!")

# Get streaming URL
print(f"View at: http://192.168.1.100:8080")

# Test robot movement
sim.move_robot(0, position=[0.5, 0.0, 0.6], smooth=True, num_waypoints=50)
sim.step(100)

print("Robot moved! Check the video stream.")

# Cleanup
sim.destroy()
```

Run it:
```bash
python test_student.py
```

## Production Deployment

### Option 1: systemd Service (Recommended)

Create a systemd service for automatic startup:

```bash
# Create service file
sudo nano /etc/systemd/system/genesis-server.service
```

Add the following content:

```ini
[Unit]
Description=Genesis Simulation Server
After=network.target

[Service]
Type=simple
User=trabalgi
WorkingDirectory=/home/trabalgi/workspace/dev/bootcamp/PYNQ_Bootcamp_Final/genesis/server
Environment="GENESIS_PORT=9002"
Environment="GENESIS_STREAM_PORT=8080"
Environment="GENESIS_BACKEND=amdgpu"
Environment="GENESIS_SHOW_VIEWER=false"
Environment="GENESIS_ADMIN_PASSWORD=bootcamp2024"
Environment="GENESIS_MAX_SESSIONS=30"
Environment="GENESIS_SESSION_TIMEOUT=7200"
ExecStart=/home/trabalgi/workspace/dev/bootcamp/PYNQ_Bootcamp_Final/genesis/server/venv/bin/python scripts/run_server.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

Enable and start the service:

```bash
# Reload systemd
sudo systemctl daemon-reload

# Enable service (auto-start on boot)
sudo systemctl enable genesis-server

# Start service
sudo systemctl start genesis-server

# Check status
sudo systemctl status genesis-server

# View logs
journalctl -u genesis-server -f
```

### Option 2: Screen/tmux Session

```bash
# Install screen
sudo apt-get install screen

# Start a screen session
screen -S genesis-server

# Load config and run server
source ~/.genesis_server_config
cd /home/trabalgi/workspace/dev/bootcamp/PYNQ_Bootcamp_Final/genesis/server
source venv/bin/activate
python scripts/run_server.py

# Detach: Ctrl+A then D
# Reattach: screen -r genesis-server
```

## Monitoring & Maintenance

### Monitor Active Sessions

```bash
# Check server status via API
curl http://localhost:9002 -X POST \
  -H "Content-Type: application/json" \
  -d '{
    "action": "admin_get_status",
    "params": {"password": "bootcamp2024"}
  }' | python -m json.tool
```

### Monitor Resources

```bash
# CPU and memory usage
htop

# GPU usage (NVIDIA)
watch -n 1 nvidia-smi

# GPU usage (AMD)
watch -n 1 rocm-smi

# Network connections
watch -n 1 'netstat -an | grep ":9002\|:8080" | wc -l'
```

### Server Logs

```bash
# If running as systemd service
journalctl -u genesis-server -f

# If running manually, output goes to terminal
# Redirect to log file:
python scripts/run_server.py > server.log 2>&1
```

### Pre-Session Checklist

**Day Before Session:**

```bash
# 1. Verify server is running
sudo systemctl status genesis-server

# 2. Test student connection workflow
python test_student.py

# 3. Clear any stale sessions (if needed)
# Use admin API to list and clean up

# 4. Check disk space
df -h

# 5. Check GPU availability
nvidia-smi  # or rocm-smi
```

**During Session:**

```bash
# Monitor connection count
watch -n 5 'netstat -an | grep :9002 | grep ESTABLISHED | wc -l'

# Monitor logs
journalctl -u genesis-server -f

# Check video stream health
curl http://localhost:8080/health
```

## Troubleshooting

### Server Won't Start

**Problem**: Port already in use
```bash
# Find what's using the port
sudo lsof -i :9002
sudo lsof -i :8080

# Kill the process
sudo kill -9 <PID>
```

**Problem**: Genesis import errors
```bash
# Verify installation
pip list | grep genesis

# Reinstall
pip uninstall genesis-world
pip install genesis-world
```

### Students Can't Connect

**Problem**: Connection refused

```bash
# Check if server is listening
netstat -tuln | grep 9002

# Check firewall
sudo ufw status

# Test from server itself
curl http://localhost:9002
```

**Problem**: Wrong network interface

```bash
# Server might be binding to wrong IP
# Check server startup output for available IPs
# Students should use the correct network IP, not localhost
```

### Video Stream Issues

**Problem**: Stream not loading

```bash
# Check stream server is running
curl http://localhost:8080/health

# Should return: {"status": "ok", ...}

# Check if student session exists
# Browse to http://SERVER_IP:8080 to see active sessions
```

**Problem**: Stream is slow/laggy

- Reduce number of concurrent students
- Check network bandwidth
- Reduce stream quality (edit stream_server.py JPEG quality parameter)

### Performance Issues

**Problem**: Server running slow with many students

```bash
# Increase MAX_SESSIONS environment variable limit
export GENESIS_MAX_SESSIONS=50

# Monitor resource usage
htop
nvidia-smi  # or rocm-smi

# Consider:
# - More RAM
# - Better GPU
# - Multiple server instances with load balancer
```

## Student Connection Info

Provide students with these details:

### For Python Code (Notebook)

```python
SERVER_IP = "192.168.1.100"  # Your actual server IP
SERVER_PORT = 9002           # Main API port
```

### For Video Streaming

```
Open browser to: http://192.168.1.100:8080

After running sim.create_environment(), you'll see your 
session appear in the list. Click it to watch your robot!
```

## Advanced Configuration

### Multiple Scenes

Scenes are defined in `scenes/` directory. Current scenes:

- `pick_and_place` - Single robot, table, 3 colored cubes
- `empty` - Single robot on plane
- `competition_card_flip` - Two robots, card flipping game

To add custom scenes, see `scenes/pick_and_place.py` for examples.

### Adjust Session Limits

```bash
# Allow more concurrent students
export GENESIS_MAX_SESSIONS=50

# Increase session timeout to 4 hours
export GENESIS_SESSION_TIMEOUT=14400
```

### Performance Tuning

```bash
# Use GPU backend for better performance
export GENESIS_BACKEND=cuda  # or amdgpu

# Disable desktop viewer on headless servers
export GENESIS_SHOW_VIEWER=false
```

## Support

For issues specific to:

- **Genesis physics engine**: https://github.com/Genesis-Embodied-AI/Genesis
- **Server setup**: Check server logs and this documentation
- **Network issues**: Coordinate with your IT department

## Quick Reference

```bash
# Start server
cd /home/trabalgi/workspace/dev/bootcamp/PYNQ_Bootcamp_Final/genesis/server
source venv/bin/activate
source ~/.genesis_server_config
python scripts/run_server.py

# Test connection
curl -X POST http://localhost:9002 -H "Content-Type: application/json" \
  -d '{"action": "create_env", "params": {"scene": "pick_and_place"}}'

# View streams
# Open browser: http://SERVER_IP:8080

# Stop server
# Press Ctrl+C (if running manually)
# OR
sudo systemctl stop genesis-server  # if using systemd
```
