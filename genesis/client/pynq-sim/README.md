# PYNQ Simulation Client

Python client library for controlling Genesis robot simulations remotely from PYNQ boards or any Python environment.

## Installation

```bash
cd genesis/client/pynq-sim
pip install -e .
```

## Quick Start

```python
from pynqsim import SimulationClient

# Connect to server (ask your instructor for IP/port)
sim = SimulationClient("192.168.1.100", port=9002)

# Create your own simulation
sim.create_environment(scene="pick_and_place")

# Move the robot
sim.move_robot(robot_id=0, position=[0.5, 0.0, 0.3])
sim.step(100)

# Clean up when done
sim.destroy()
```

## Available Scenes

| Scene | Description |
|-------|-------------|
| `empty` | Single robot arm on plane |
| `grid_5x6` | Robot with 5×6 colored grid |
| `pick_and_place` | Robot with table and cubes |

Competition scenes (`competition_2v2`, `competition_card_flip`) are started by instructors only.

## Robot Control

### Moving the Robot

```python
# Move end effector to position (inverse kinematics)
sim.move_robot(robot_id=0, position=[0.5, 0.0, 0.4])
sim.step(100)

# Move joints directly (radians)
sim.move_joints(robot_id=0, angles=[0, 0.5, 0, -1.5, 0, 1.0, 0.5])
sim.step(100)

# Smooth motion (uses motion planning)
sim.move_robot(robot_id=0, position=[0.6, 0.1, 0.3], smooth=True)
```

### Gripper Control

```python
# Open gripper
sim.open_gripper(robot_id=0)
sim.step(50)

# Close gripper
sim.close_gripper(robot_id=0)
sim.step(50)
```

### Getting Robot State

```python
state = sim.get_state(robot_id=0)
print(f"Position: {state['end_effector']['position']}")
print(f"Joints: {state['joints']}")
```

## Adding Objects

```python
# Add a cube
cube_id = sim.add_cube(position=[0.5, 0.0, 0.1], size=[0.04, 0.04, 0.04])

# Add a sphere
sphere_id = sim.add_sphere(position=[0.4, 0.1, 0.1], radius=0.03)

# Add a cylinder
cyl_id = sim.add_cylinder(position=[0.6, -0.1, 0.1], radius=0.02, height=0.1)

# List all objects
objects = sim.get_objects()
```

## Recording Video

```python
# Start recording
sim.start_recording()

# Perform actions
sim.move_robot(robot_id=0, position=[0.5, 0.0, 0.4])
sim.step(100)

# Stop and display in Jupyter
video = sim.stop_recording()
sim.show_video(video)
```

## Live Viewing

View your robot in real-time at: `http://<server-ip>:8080/view/<your-token>`

The link is automatically displayed when you create an environment in Jupyter.

## Competition Mode

Join instructor-hosted competitions for turn-based card flip games.

### Joining a Competition

```python
# Instructor starts competition first
# Then you join with your team ID
sim.join_competition(team_id="team_red")

# Move your robot (no step() needed - auto-stepped by server)
sim.move_robot(position=[0.3, 0, 0.2])

# Check game state
state = sim.get_competition_state()
print(f"Current turn: {state['current_turn']}")
print(f"Scores: {state['scores']}")
```

### Playing the Card Game

```python
# Flip a card (row, col)
result = sim.flip_card(row=0, col=2)
print(f"Flipped color: {result['color_idx']}")

if result.get('match_result', {}).get('matched'):
    print("Match! Go again!")
elif result.get('match_result', {}).get('turn_ended'):
    print("No match. Turn passes.")

# End your turn manually
sim.end_turn()

# Get card state
card = sim.get_card_state(row=0, col=2)
print(f"Flipped: {card['flipped']}, Matched: {card['matched']}")

# Get full grid
grid = sim.get_grid_state()  # Returns 6×5 grid of card states
```

### Leaving Competition

```python
sim.leave_competition()
```

## Admin Commands

**For instructors only** - requires admin password.

```python
# Start competition
sim.admin_start_competition(
    scene="competition_card_flip",
    password="admin123",
    card_layout={"grid": [...]}  # Optional custom layout
)

# Check server status
status = sim.admin_get_status(password="admin123")
print(f"Active sessions: {status['sessions']}")

# Stop competition
sim.admin_stop_competition(password="admin123")

# List available card images
images = sim.admin_list_card_images(password="admin123")

# Reset board (re-cover all cards, zero scores)
sim.admin_reset_board(password="admin123")
```

## Advanced Usage

### Direct Server Requests

For features not wrapped in the client library:

```python
# Example: flip_raw (admin flip without scoring)
result = sim._request("flip_raw", {
    "row": 0,
    "col": 2,
    "robot_id": 0  # Which arm to use
})

# Example: unflip_raw (instant re-cover)
result = sim._request("unflip_raw", {"row": 0, "col": 2})
```

### Password-Protected Competitions

Instructors can require passwords for teams:

```python
# Admin starts with team passwords
sim.admin_start_competition(
    scene="competition_card_flip",
    password="admin123",
    join_passwords={
        "team_red": "red_secret",
        "team_blue": "blue_secret"
    }
)

# Students join with their team password
sim.join_competition(team_id="team_red", password="red_secret")
```

## Troubleshooting

**Connection refused:**
- Check server IP and port
- Ensure server is running: `python -m genesis_server.server`
- Check firewall settings

**"Backend not available" error:**
- Server GPU backend not working
- Ask instructor to restart with: `GENESIS_BACKEND=cpu python -m genesis_server.server`

**Slow performance:**
- Server may be using CPU backend (expected on machines without GPU)
- Reduce `smooth=True` waypoints or avoid smooth motion

## Examples

See notebooks in `notebooks/`:
- `PYNQ_RemoteSim_GettingStarted.ipynb` - Complete tutorial
- `PYNQ_Competition_CardFlip.ipynb` - Card flip competition guide
- `PoC_TeamRed.ipynb` / `PoC_TeamBlue.ipynb` - Competition examples
