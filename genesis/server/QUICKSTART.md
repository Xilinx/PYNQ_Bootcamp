# Genesis Server - Quick Start

Get your Genesis server running in 5 minutes.

## Prerequisites

- Linux machine with Python 3.8+
- Network access from student machines
- (Optional) NVIDIA or AMD GPU

## Installation

```bash
# 1. Navigate to server directory
cd /home/trabalgi/workspace/dev/bootcamp/PYNQ_Bootcamp_Final/genesis/server

# 2. Create virtual environment
python3 -m venv venv
source venv/bin/activate

# 3. Install dependencies
pip install -e .
```

## Configuration

```bash
# Set essential environment variables
export GENESIS_PORT=9002              # API port
export GENESIS_STREAM_PORT=8080       # Video streaming port
export GENESIS_BACKEND=cpu            # or: cuda, amdgpu, gpu
export GENESIS_SHOW_VIEWER=false      # Disable GUI on headless server
export GENESIS_MAX_SESSIONS=30        # Max students
export GENESIS_ADMIN_PASSWORD=bootcamp2024
```

## Run Server

```bash
# Activate virtual environment (if not already)
source venv/bin/activate

# Start server
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

## Test It Works

### Test 1: API Server

```bash
# Open new terminal
curl -X POST http://localhost:9002 \
  -H "Content-Type: application/json" \
  -d '{"action": "create_env", "params": {"scene": "pick_and_place"}}'

# Should return: {"token": "...", "status": "ok"}
```

### Test 2: Video Stream

Open browser to: `http://YOUR_SERVER_IP:8080`

You should see the Genesis Live Viewer interface.

## Student Setup

Students need these values in their notebook:

```python
SERVER_IP = "192.168.1.100"  # Your actual server IP from startup output
SERVER_PORT = 9002
```

Video stream URL: `http://192.168.1.100:8080`

## Firewall Setup

```bash
# Allow required ports
sudo ufw allow 9002/tcp  # API
sudo ufw allow 8080/tcp  # Video
```

## Troubleshooting

**Port already in use?**
```bash
sudo lsof -i :9002
sudo lsof -i :8080
# Kill the process using the port
```

**Can't connect from student machines?**
- Check firewall: `sudo ufw status`
- Get correct IP: `hostname -I | awk '{print $1}'`
- Verify server is listening: `netstat -tuln | grep 9002`

**Genesis import error?**
```bash
pip install --upgrade genesis-world
```

## Next Steps

- See [SETUP.md](SETUP.md) for production deployment with systemd
- See [README.md](README.md) for API reference and available scenes

## Quick Commands

```bash
# Start server
source venv/bin/activate && python scripts/run_server.py

# Check if running
curl http://localhost:9002 -X POST -H "Content-Type: application/json" \
  -d '{"action": "create_env", "params": {"scene": "pick_and_place"}}'

# View logs (if using systemd)
journalctl -u genesis-server -f

# Get server IP
hostname -I | awk '{print $1}'
```

## Architecture

```
Student Laptop               Server Machine
┌─────────────┐             ┌──────────────────────┐
│             │   9002      │  API Server          │
│ Jupyter ────┼────────────►│  (Robot control)     │
│ Notebook    │             │                      │
│             │   8080      │  Stream Server       │
│ Browser ────┼────────────►│  (Live video)        │
└─────────────┘             └──────────────────────┘
```

That's it! Your server is ready for the bootcamp.
