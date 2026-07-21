# Card Images for Competition

Place PNG or JPG images here for the card flip competition game.

## Requirements

- **Format:** PNG or JPG
- **Aspect ratio:** Square recommended (e.g., 256×256, 512×512)
- **Naming:** Descriptive filenames (e.g., `cat.png`, `robot.png`, `star.jpg`)

## How It Works

The competition scene pairs images for the memory game:
- **Default mode:** Server randomly picks images and assigns pairs
- **Custom layout mode:** Admin specifies exact image placement via `admin_start_competition(card_layout=...)`

## Grid Size

Standard competition uses a **6×5 grid** (30 cards = 15 pairs).

You need **at least 15 different images** for a full game.

## Suggested Images

Organize by theme for easier gameplay:

**Animals:**
- `cat.png`, `dog.png`, `bird.png`, `fish.png`, `cow.png`, etc.

**Objects:**
- `star.png`, `moon.png`, `heart.png`, `car.png`, `bicycle.png`, etc.

**Robots:**
- `robot1.png`, `robot2.png`, `arm.png`, `gripper.png`, etc.

**Shapes:**
- `circle.png`, `square.png`, `triangle.png`, `hexagon.png`, etc.

## Custom Layouts

Admins can specify exact image placement:

```python
from pynqsim import SimulationClient
sim = SimulationClient("server-ip", 9002)

# Define a 6×5 grid (30 cards, 15 pairs)
layout = {
    "grid": [
        ["cat", "dog", "bird", "cat", "fish"],     # Row 0
        ["star", "moon", "dog", "star", "bird"],   # Row 1
        ["car", "bike", "fish", "moon", "heart"],  # Row 2
        ["robot", "arm", "bike", "car", "gripper"],# Row 3
        ["circle", "square", "arm", "circle", "robot"], # Row 4
        ["heart", "triangle", "square", "gripper", "triangle"], # Row 5
    ]
}

sim.admin_start_competition(
    scene="competition_card_flip",
    password="admin123",
    card_layout=layout
)
```

Each label (e.g., `"cat"`) must appear **exactly twice** in the grid.

## Adding Images

1. Drop image files into this directory: `genesis/server/assets/card_images/`
2. Restart the server (it scans this directory on startup)
3. Use `sim.admin_list_card_images(password="admin123")` to verify

Images are automatically available for competitions!
