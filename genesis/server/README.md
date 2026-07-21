# Genesis Simulation Server

HTTP server providing remote robot control via Genesis physics simulations with live video streaming.

## Quick Start

```bash
# Install server
cd genesis/server
pip install -e .

# Start server
python -m genesis_server.server
```

Server will display available IP addresses and ports on startup.

## Configuration

Set environment variables before starting:

| Variable | Default | Description |
|----------|---------|-------------|
| `GENESIS_PORT` | 9002 | Main API port |
| `GENESIS_STREAM_PORT` | 8080 | Video streaming port |
| `GENESIS_BACKEND` | amdgpu | Backend: `cpu`, `cuda`, `amdgpu`, `metal` |
| `GENESIS_SHOW_VIEWER` | true | Enable live viewer |
| `GENESIS_ADMIN_PASSWORD` | admin123 | Admin password |
| `GENESIS_MAX_SESSIONS` | 10 | Max concurrent sessions |
| `GENESIS_SESSION_TIMEOUT` | 7200 | Session timeout (seconds) |

### Backend Selection

Choose based on your hardware:

```bash
# CPU (any machine, slower)
GENESIS_BACKEND=cpu python -m genesis_server.server

# AMD GPU with ROCm
GENESIS_BACKEND=amdgpu python -m genesis_server.server

# NVIDIA GPU with CUDA
GENESIS_BACKEND=cuda python -m genesis_server.server

# Apple Silicon
GENESIS_BACKEND=metal python -m genesis_server.server
```

**Troubleshooting GPU:**
- AMD: Install ROCm-enabled PyTorch: `pip install torch --index-url https://download.pytorch.org/whl/rocm6.0`
- NVIDIA: Install CUDA-enabled PyTorch: `pip install torch --index-url https://download.pytorch.org/whl/cu121`
- If GPU not detected, may need: `export HSA_OVERRIDE_GFX_VERSION=10.3.0` (AMD) or add user to `render`/`video` groups

## Available Scenes

| Scene | Description | Robots |
|-------|-------------|--------|
| `empty` | Plane with single Franka arm | 1 |
| `grid_5x6` | Franka arm with 5×6 colored grid | 1 |
| `pick_and_place` | Franka arm with table and cubes | 1 |
| `competition_2v2` | Two Franka arms for free competition | 2 |
| `competition_card_flip` | Two arms, 6×5 memory card game | 2 |

## API Overview

### Standard Mode

Single-user simulation with full control:

- `create_env` - Create isolated simulation
- `add_object` - Add cubes, spheres, cylinders
- `move_robot` - Move via inverse kinematics
- `move_joints` - Direct joint control
- `gripper` - Open/close gripper
- `get_state` - Get robot state
- `step` - Advance simulation
- `reset` - Reset to initial state
- `start_recording` / `stop_recording` - Video capture
- `destroy_env` - Clean up

### Competition Mode

Multi-user turn-based card flip game:

**Player Actions:**
- `join_competition` - Join with team ID (optional password)
- `flip_card` - Flip a card (memory game rules)
- `get_competition_state` - Get scores, current turn, board state
- `get_card_state` / `get_grid_state` - Inspect cards
- `end_turn` - Pass turn to other team
- `leave_competition` - Exit competition

**Admin Actions:**
- `admin_start_competition` - Start game with scene and optional passwords
- `admin_stop_competition` - End game
- `admin_reset_board` - Re-cover all cards, zero scores
- `admin_cover_card` - Force re-cover single card
- `admin_get_status` - Server status
- `admin_list_card_images` - List available card images
- `admin_set_card_layout` - Custom card layout

**Advanced Actions:**
- `flip_raw` - Flip card without scoring/turn logic (admin)
- `unflip_raw` - Re-cover card instantly (admin)

## Live Video Streaming

Access the live viewer at: `http://<server-ip>:8080`

- Auto-updates at ~30 FPS
- Works in any browser
- No additional setup required

## Extending

### Adding Custom Scenes

Create `scenes/my_scene.py`:

```python
import genesis as gs

def setup(scene, card_layout=None):
    """Setup function - always include card_layout parameter for compatibility."""
    plane = scene.add_entity(gs.morphs.Plane())
    robot = scene.add_entity(
        gs.morphs.MJCF(file="xml/franka_emika_panda/panda.xml")
    )
    return {"robots": [robot]}
```

Use with: `sim.create_environment(scene="my_scene")`

### Adding Card Images

Drop PNG/JPG images into `assets/card_images/` - they'll be automatically available for the card flip competition. See `assets/card_images/README.md` for details.

## Architecture

- **HTTP Server** (`genesis_server/server.py`) - Main request handler
- **Flask Stream Server** (`genesis_server/stream_server.py`) - Live video streaming
- **Competition Manager** (`genesis_server/competition.py`) - Turn-based game logic
- **Simulation** (`genesis_server/simulation.py`) - Genesis physics wrapper
- **Scenes** (`scenes/*.py`) - Scene definitions

All simulation state is managed server-side; clients send commands via HTTP POST with JSON payloads.
