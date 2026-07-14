import genesis as gs


def setup(scene):
    """Default scene: plane + single Franka arm at origin."""
    plane = scene.add_entity(gs.morphs.Plane())

    franka = scene.add_entity(
        gs.morphs.MJCF(file="xml/franka_emika_panda/panda.xml")
    )

    return {"robots": [franka]}
