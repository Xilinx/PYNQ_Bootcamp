# Genesis Remote Robotics

Remote robot simulation system using Genesis physics engine with live video streaming.

## Quick Start

### Server Setup

```bash
# Install
cd genesis/server
pip install -e .

# Run (AMD GPU)
python -m genesis_server.server
```

Server displays available IPs on startup. Share the IP with students.

### Client Usage

```bash
# Install
cd genesis/client/pynq-sim
pip install -e .
```

```python
from pynqsim import SimulationClient

# Connect (ask instructor for IP)
sim = SimulationClient("192.168.1.100", port=9002)

# Create environment
sim.create_environment(scene="pick_and_place")

# Move robot
sim.move_robot(robot_id=0, position=[0.5, 0.0, 0.3])
sim.step(100)

# Clean up
sim.destroy()
```

## Configuration

Set before starting server:

| Variable | Default | Description |
|----------|---------|-------------|
| `GENESIS_PORT` | 9002 | API port |
| `GENESIS_STREAM_PORT` | 8080 | Video streaming port |
| `GENESIS_BACKEND` | amdgpu | Use `cpu` if GPU fails |
| `GENESIS_ADMIN_PASSWORD` | admin123 | Admin password |

### GPU Setup (AMD)

```bash
# Install ROCm PyTorch
pip install torch --index-url https://download.pytorch.org/whl/rocm6.0

# If GPU not detected, set:
export HSA_OVERRIDE_GFX_VERSION=10.3.0  # Adjust for your GPU
sudo usermod -a -G render,video $USER   # Then logout/login

# Start with CPU fallback if issues:
GENESIS_BACKEND=cpu python -m genesis_server.server
```

## Available Scenes

- `empty` - Single robot arm
- `pick_and_place` - Robot with table and cubes
- `grid_5x6` - Robot with colored grid
- `competition_card_flip` - Two-player memory card game

## Competition Mode

### Instructor: Start Competition

```python
sim = SimulationClient("server-ip", 9002)

# Simple start
sim.admin_start_competition("competition_card_flip", password="admin123")

# With team passwords
sim.admin_start_competition(
    "competition_card_flip",
    password="admin123",
    join_passwords={"team_red": "secret1", "team_blue": "secret2"}
)
```

### Students: Join and Play

```python
# Join team
sim.join_competition(team_id="team_red")
# or with password: sim.join_competition("team_red", password="secret1")

# Flip cards
result = sim.flip_card(row=0, col=2)
if result.get('match_result', {}).get('matched'):
    print("Match! Go again!")

# Check state
state = sim.get_competition_state()
print(f"Scores: {state['scores']}")
```

### Admin Controls

```python
# Reset board (re-cover all cards, zero scores)
sim.admin_reset_board(password="admin123")

# Stop competition
sim.admin_stop_competition(password="admin123")
```

## Live Viewer

Open in browser: `http://<server-ip>:8080`

## Adding Custom Scenes

Create `genesis/server/scenes/my_scene.py`:

```python
import genesis as gs

def setup(scene, card_layout=None):
    plane = scene.add_entity(gs.morphs.Plane())
    robot = scene.add_entity(gs.morphs.MJCF(file="xml/franka_emika_panda/panda.xml"))
    return {"robots": [robot]}
```

## Card Images

Add PNG/JPG images to `genesis/server/assets/card_images/` for the card flip game.

For 6×5 grid (30 cards), need 15 unique images (each appears twice).

## Troubleshooting

**"Backend not available":**
- Check PyTorch sees GPU: `python -c "import torch; print(torch.cuda.is_available())"`
- Use CPU: `GENESIS_BACKEND=cpu python -m genesis_server.server`

**Connection refused:**
- Check server is running
- Verify IP/port
- Check firewall

**GPU not detected:**
- Run `rocm-smi` to verify GPU visible
- Set `HSA_OVERRIDE_GFX_VERSION` for your GPU model
- Add user to render/video groups

## Examples

See `genesis/client/` for Jupyter notebooks:
- `PYNQ_RemoteSim_GettingStarted.ipynb` - Tutorial
- `PYNQ_Competition_CardFlip.ipynb` - Competition guide
- `PoC_TeamRed.ipynb` / `PoC_TeamBlue.ipynb` - Competition examples
