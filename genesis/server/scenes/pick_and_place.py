import genesis as gs


def setup(scene):
    """Franka arm with table and cubes for pick-and-place tasks.0707"""
    plane = scene.add_entity(gs.morphs.Plane())

    franka = scene.add_entity(
        gs.morphs.MJCF(file="xml/franka_emika_panda/panda.xml")
    )

    # Add table
    table = scene.add_entity(
        gs.morphs.Box(
            size=(0.6, 0.4, 0.02),
            pos=(0.5, 0.0, 0.4),
            fixed=True,
        ),
        surface=gs.surfaces.Plastic(color=(0.6, 0.4, 0.2, 1.0)),  # Brown
    )

    # Add table legs
    leg_positions = [
        (0.25, -0.15, 0.2),
        (0.25, 0.15, 0.2),
        (0.75, -0.15, 0.2),
        (0.75, 0.15, 0.2),
    ]
    for pos in leg_positions:
        scene.add_entity(
            gs.morphs.Box(
                size=(0.03, 0.03, 0.4),
                pos=pos,
                fixed=True,
            ),
            surface=gs.surfaces.Plastic(color=(0.6, 0.4, 0.2, 1.0)),
        )

    # Add cubes on table
    cubes = []
    cube_positions = [
        (0.5, -0.1, 0.43),
        (0.5, 0.0, 0.43),
        (0.5, 0.1, 0.43),
    ]
    cube_colors = [
        (1.0, 0.0, 0.0, 1.0),  # Red
        (1.0, 0.0, 0.0, 1.0),  # Green
        (1.0, 0.0, 0.0, 1.0),  # Blue
    ]

    for pos, color in zip(cube_positions, cube_colors):
        cube = scene.add_entity(
            gs.morphs.Box(
                size=(0.04, 0.04, 0.04),
                pos=pos,
                fixed=False,
            ),
            surface=gs.surfaces.Plastic(color=color),
        )
        cubes.append(cube)

    return {"robots": [franka], "objects": cubes}
