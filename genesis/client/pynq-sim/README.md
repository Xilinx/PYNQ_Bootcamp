# PYNQ Simulation Client (pynq-sim)

Client library for controlling Genesis simulations from PYNQ boards.

## Installation

```bash
cd pynq-sim
pip install -e .
```

## Quick Start

```python
from pynqsim import SimulationClient

# Connect to server
sim = SimulationClient("192.168.1.100")

# Create environment
sim.create_environment(scene="empty")

# Move robot
sim.move_robot(0, position=[0.5, 0.0, 0.3])
sim.step(100)

# Clean up
sim.destroy()
```

## Available Scenes

- `empty` - Just a robot arm
- `grid_5x6` - Robot with colored grid
- `pick_and_place` - Robot with table and cubes

## Robot Control

```python
# Move end effector to position (uses inverse kinematics)
sim.move_robot(0, position=[0.5, 0.0, 0.4])
sim.step(100)

# Move joints directly
sim.move_joints(0, angles=[0, 0.5, 0, -1.5, 0, 1.0, 0.5])
sim.step(100)

# Gripper control
sim.open_gripper(0)
sim.step(50)
sim.close_gripper(0)
sim.step(50)

# Get robot state
state = sim.get_state(0)
print(f"Position: {state['end_effector']['position']}")
```

## Adding Objects

```python
# Add a cube
cube_id = sim.add_cube([0.5, 0.0, 0.1], size=[0.04, 0.04, 0.04])

# Add a sphere
sphere_id = sim.add_sphere([0.4, 0.1, 0.1], radius=0.03)

# Add a cylinder
cyl_id = sim.add_cylinder([0.6, -0.1, 0.1], radius=0.02, height=0.1)
```

## Recording Video

```python
sim.start_recording()

# Do robot movements...
sim.move_robot(0, position=[0.5, 0.0, 0.4])
sim.step(100)

video = sim.stop_recording()
sim.show_video(video)  # Display in Jupyter
```

## Competition Mode

```python
# Join a competition (admin must start it first)
sim.join_competition(team_id="team_red")

# Move your robot (no step() needed - server auto-steps)
sim.move_robot(position=[0.3, 0, 0.2])

# Check game state
state = sim.get_competition_state()
print(f"Current turn: {state['current_turn']}")
print(f"Scores: {state['scores']}")

# End your turn
sim.end_turn()

# Leave competition
sim.leave_competition()
```

## Admin Commands (Instructors Only)

```python
# Start competition
sim.admin_start_competition("competition_card_flip", password="admin123")

# Check server status
status = sim.admin_get_status(password="admin123")
print(f"Active sessions: {status['sessions']}")

# Stop competition
sim.admin_stop_competition(password="admin123")
```

See `notebooks/PYNQ_RemoteSim_GettingStarted.ipynb` for a complete tutorial.
