import genesis as gs


def setup(scene):
    """Two Franka arms for free-form competition."""
    plane = scene.add_entity(gs.morphs.Plane())

    # Robot for team red
    robot_red = scene.add_entity(
        gs.morphs.MJCF(file="xml/franka_emika_panda/panda.xml"),
    )

    # Robot for team blue
    robot_blue = scene.add_entity(
        gs.morphs.MJCF(file="xml/franka_emika_panda/panda.xml"),
    )

    return {
        "robots": [robot_red, robot_blue],
    }
