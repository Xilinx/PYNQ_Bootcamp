# Genesis Simulation Server

HTTP server that manages Genesis physics simulations for remote robot control.

## Installation

```bash
cd genesis_server
pip install -e .
```

## Running the Server

```bash
# Using the entry point script
python scripts/run_server.py

# Or as a module
python -m genesis_server.server
```

## Configuration

Set environment variables before starting:

| Variable | Default | Description |
|----------|---------|-------------|
| `GENESIS_PORT` | 9002 | Server port |
| `GENESIS_BACKEND` | cpu | Backend: `cpu`, `gpu`, `cuda`, `amdgpu`, `metal` |
| `GENESIS_ADMIN_PASSWORD` | admin123 | Admin password |
| `GENESIS_MAX_SESSIONS` | 10 | Max concurrent sessions |
| `GENESIS_SESSION_TIMEOUT` | 7200 | Session timeout (seconds) |

## Available Scenes

- `empty` - Plane with single Franka arm
- `grid_5x6` - Franka arm with 5x6 colored grid
- `pick_and_place` - Franka arm with table and cubes
- `competition_2v2` - Two Franka arms for competition
- `competition_card_flip` - Two arms with 5x6 flippable cards (turn-based)

## Adding Custom Scenes

Drop Python files in the `scenes/` folder. Each file must have a `setup(scene)` function:

```python
# scenes/my_scene.py
import genesis as gs

def setup(scene):
    plane = scene.add_entity(gs.morphs.Plane())
    robot = scene.add_entity(gs.morphs.MJCF(file="xml/franka_emika_panda/panda.xml"))
    return {"robots": [robot]}
```

The scene becomes available as `scene="my_scene"`.

## Adding Card Images

For the card flip competition, add PNG/JPG images to `assets/card_images/`.

## API Reference

### Standard Mode Actions

| Action | Params | Response |
|--------|--------|----------|
| `create_env` | `scene` | `token`, `status` |
| `add_object` | `type`, `position`, `size`/`radius` | `object_id` |
| `move_robot` | `robot_id`, `position` | `joint_angles` |
| `move_joints` | `robot_id`, `angles` | `status` |
| `gripper` | `robot_id`, `action` | `status` |
| `get_state` | `robot_id` | `joints`, `end_effector` |
| `step` | `steps` | `status` |
| `reset` | - | `status` |
| `start_recording` | - | `status` |
| `stop_recording` | - | `video_base64` |
| `get_objects` | - | `objects` |
| `destroy_env` | - | `status` |

### Competition Mode Actions

| Action | Params | Response |
|--------|--------|----------|
| `join_competition` | `team_id` | `token` |
| `leave_competition` | - | `status` |
| `get_competition_state` | - | `robots`, `objects`, `current_turn`, `scores` |
| `end_turn` | - | `next_turn` |

### Admin Actions

| Action | Params | Response |
|--------|--------|----------|
| `admin_get_status` | `password` | `sessions`, `competition_active` |
| `admin_start_competition` | `password`, `scene`, `card_layout` | `status` |
| `admin_stop_competition` | `password` | `status` |
| `admin_list_card_images` | `password` | `images` |
