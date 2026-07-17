import genesis as gs


def setup(scene, card_layout=None):
    """Simple scene: Franka arm with table and three colored cubes.

    Args:
        scene: Genesis scene object
        card_layout: Optional card layout (not used, for compatibility)
    """
    plane = scene.add_entity(gs.morphs.Plane())

    franka = scene.add_entity(
        gs.morphs.MJCF(file="xml/franka_emika_panda/panda.xml")
    )

    # Add three cubes on ground
    cubes = []
    cube_positions = [
        (0.5, -0.1, 0.02),
        (0.5, 0.0, 0.02),
        (0.5, 0.1, 0.02),
    ]
    cube_colors = [
        (1.0, 0.0, 0.0, 1.0),  # Red
        (0.0, 1.0, 0.0, 1.0),  # Green
        (0.0, 0.0, 1.0, 1.0),  # Blue
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
