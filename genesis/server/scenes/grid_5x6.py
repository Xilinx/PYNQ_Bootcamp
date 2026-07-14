import genesis as gs


def setup(scene):
    """Franka arm facing 5x6 grid of colored squares."""
    plane = scene.add_entity(gs.morphs.Plane())

    franka = scene.add_entity(
        gs.morphs.MJCF(file="xml/franka_emika_panda/panda.xml")
    )

    # Create 5x6 grid of colored tiles
    colors = [
        (1.0, 0.0, 0.0, 1.0),  # Red
        (0.0, 1.0, 0.0, 1.0),  # Green
        (0.0, 0.0, 1.0, 1.0),  # Blue
        (1.0, 1.0, 0.0, 1.0),  # Yellow
        (1.0, 0.0, 1.0, 1.0),  # Magenta
        (0.0, 1.0, 1.0, 1.0),  # Cyan
    ]

    grid_start = (0.3, -0.15)
    tile_size = (0.05, 0.05, 0.002)
    spacing = 0.06

    for row in range(5):
        for col in range(6):
            x = grid_start[0] + col * spacing
            y = grid_start[1] + row * spacing
            color = colors[(row + col) % len(colors)]

            scene.add_entity(
                gs.morphs.Box(
                    size=tile_size,
                    pos=(x, y, 0.001),
                    fixed=True,
                ),
                surface=gs.surfaces.Plastic(color=color),
            )

    return {"robots": [franka]}
