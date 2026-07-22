import genesis as gs


def setup(scene, card_layout=None):
    """Default scene: plane + single Franka arm at origin.

    Args:
        scene: Genesis scene object
        card_layout: Optional card layout (not used, for compatibility)
    """
    plane = scene.add_entity(gs.morphs.Plane())

    franka = scene.add_entity(
        gs.morphs.MJCF(file="xml/franka_emika_panda/panda.xml")
    )

    return {"robots": [franka]}
