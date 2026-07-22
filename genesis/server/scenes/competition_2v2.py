import genesis as gs


def setup(scene, card_layout=None):
    """Two Franka arms for free-form competition.

    Args:
        scene: Genesis scene object
        card_layout: Optional card layout (not used, for compatibility)
    """
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
