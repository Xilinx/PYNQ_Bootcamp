import genesis as gs
import os
import random

# Import config at module level with fallback
try:
    from genesis_server.config import CARD_IMAGES_DIR
except ImportError:
    CARD_IMAGES_DIR = os.path.join(os.path.dirname(__file__), "..", "assets", "card_images")


def setup(scene, card_layout=None):
    """Two Franka arms on OPPOSITE sides of a card grid.

    Layout:
    - Red robot on LEFT side, facing +X toward grid
    - Blue robot on RIGHT side, facing -X toward grid
    - 6x5 grid of cards in the middle (30 cards, 15 color pairs)

    Each card position has TWO stacked boxes:
    - Bottom: Colored card (fixed=True) - the actual color to reveal
    - Top: Gray cover (fixed=False) - physics-enabled, can be picked up
    """
    # Solid, opaque ground slab instead of an infinitely-thin Plane.
    # Its top face is at z=0. The recessed colored cards live INSIDE this
    # slab, so they can't be seen from the sides or from below the floor -
    # the only colored surface exposed at the surface is each card's top
    # face, which stays hidden under its gray cover until it's lifted.
    ground_thickness = 0.20
    plane = scene.add_entity(
        gs.morphs.Box(
            size=(10.0, 10.0, ground_thickness),
            pos=(0.0, 0.0, -ground_thickness / 2),  # top face at z=0
            fixed=True,
        ),
        surface=gs.surfaces.Plastic(color=(0.2, 0.2, 0.25, 1.0)),  # dark gray floor
    )

    # Grid dimensions - sized for robot reach (~0.6m comfortable reach)
    cover_size = 0.04           # 4cm gray cover (same as working cube - fits in gripper)
    card_size = 0.035      # Colored card matches cover footprint so it's fully hidden
    colored_thickness = 0.005   # 5mm colored card (bottom layer)
    cover_thickness = 0.04      # 40mm gray cover (same as working cube example)
    card_spacing = 0.10         # 10cm between card centers (tightened for reach)
    grid_cols = 5
    grid_rows = 6
    grid_width = grid_cols * card_spacing   # 0.50m (X span between robots - smaller = easier reach)
    grid_height = grid_rows * card_spacing  # 0.60m (Y span)

    # Grid center position
    grid_center_x = 0.55  # Center of grid
    grid_center_y = 0.0

    # Robot distance from grid edge
    robot_offset = 0.18  # Distance from grid edge to robot base (closer for reach)

    # Two robot arms on OPPOSITE sides of the grid
    # Team red robot on the LEFT side (facing right toward grid)
    robot_red_x = grid_center_x - grid_width/2 - robot_offset
    robot_red_pos = (robot_red_x, 0, 0)
    robot_red = scene.add_entity(
        gs.morphs.MJCF(
            file="xml/franka_emika_panda/panda.xml",
            pos=robot_red_pos,
            euler=(0, 0, 0),  # Facing +X (toward grid)
        ),
    )

    # Red marker cube to identify team red robot
    scene.add_entity(
        gs.morphs.Box(
            size=(0.06, 0.06, 0.06),
            pos=(robot_red_pos[0] - 0.20, robot_red_pos[1], 0.03),
            fixed=True,
        ),
        surface=gs.surfaces.Plastic(color=(1.0, 0.0, 0.0, 1.0)),  # Red
    )

    # Team blue robot on the RIGHT side (facing left toward grid)
    robot_blue_x = grid_center_x + grid_width/2 + robot_offset
    robot_blue_pos = (robot_blue_x, 0, 0)
    robot_blue = scene.add_entity(
        gs.morphs.MJCF(
            file="xml/franka_emika_panda/panda.xml",
            pos=robot_blue_pos,
            euler=(0, 0, 180),  # Rotated 180 degrees to face -X (toward grid)
        ),
    )

    # Blue marker cube to identify team blue robot
    scene.add_entity(
        gs.morphs.Box(
            size=(0.06, 0.06, 0.06),
            pos=(robot_blue_pos[0] + 0.20, robot_blue_pos[1], 0.03),
            fixed=True,
        ),
        surface=gs.surfaces.Plastic(color=(0.0, 0.0, 1.0, 1.0)),  # Blue
    )

    # Back color for face-down cards
    back_color = (0.03, 0.03, 0.03, 1.0)  # Near-black cover

    # Create grid of cards
    cards = []

    # Grid starting position (bottom-left corner of grid)
    grid_start_x = grid_center_x - grid_width / 2 + card_spacing / 2
    grid_start_y = grid_center_y - grid_height / 2 + card_spacing / 2

    # Card colors - 15 distinct colors for 15 pairs (30 cards)
    card_colors = [
        (1.0, 0.0, 0.0, 1.0),   # 0  Red
        (0.0, 0.8, 0.0, 1.0),   # 1  Green
        (0.0, 0.0, 1.0, 1.0),   # 2  Blue
        (1.0, 0.9, 0.0, 1.0),   # 3  Yellow
        (1.0, 0.5, 0.0, 1.0),   # 4  Orange
        (0.5, 0.0, 1.0, 1.0),   # 5  Purple
        (0.36, 0.18, 0.0, 1.0), # 6  Brown
        (1.0, 0.4, 0.7, 1.0),   # 7  Pink
        (0.0, 0.8, 0.8, 1.0),   # 8  Cyan
        (1.0, 0.0, 1.0, 1.0),   # 9  Magenta
        (0.6, 0.8, 0.2, 1.0),   # 10 Lime
        (0.0, 0.4, 0.4, 1.0),   # 11 Teal
        (0.5, 0.5, 0.5, 1.0),   # 12 Gray
        (0.5, 0.0, 0.0, 1.0),   # 13 Maroon
        (0.0, 0.0, 0.5, 1.0),   # 14 Navy
    ]

    num_cards = grid_rows * grid_cols  # 30 cards
    num_pairs = num_cards // 2  # 15 pairs

    # ------------------------------------------------------------------
    # Determine each cell's color_idx: either from an admin-supplied
    # layout (a 6x5 grid of NAMES) or, if none/invalid, a random shuffle.
    #
    # card_layout format (admin-authored):
    #   {"grid": [[name, name, ...5...], ...6 rows...]}
    # or just the bare 2D list [[name,...],...]. Each unique name is
    # assigned a stable color_idx by first appearance. To play as a
    # memory game every name should appear exactly twice, but that is
    # not enforced here -- unmatched/extra names still render, they just
    # won't have a partner to match.
    # ------------------------------------------------------------------
    def _random_pairs():
        pairs = list(range(num_pairs)) * 2
        random.shuffle(pairs)
        return pairs, {}

    def _resolve_layout(layout):
        """Return (color_pairs_flat, name_by_idx) from a names grid, or None."""
        grid = layout.get("grid") if isinstance(layout, dict) else layout
        if not isinstance(grid, (list, tuple)):
            return None
        # Flatten row-major and validate dimensions.
        flat = []
        for r in range(grid_rows):
            if r >= len(grid) or not isinstance(grid[r], (list, tuple)):
                return None
            row_vals = grid[r]
            for c in range(grid_cols):
                if c >= len(row_vals):
                    return None
                flat.append(str(row_vals[c]))
        if len(flat) != num_cards:
            return None
        # Assign a color_idx to each unique name by first appearance.
        name_to_idx = {}
        idx_to_name = {}
        pairs = []
        for name in flat:
            if name not in name_to_idx:
                new_idx = len(name_to_idx)
                if new_idx >= len(card_colors):
                    # More unique names than we have colors -> bail to random.
                    return None
                name_to_idx[name] = new_idx
                idx_to_name[new_idx] = name
            pairs.append(name_to_idx[name])
        return pairs, idx_to_name

    color_pairs = None
    name_by_idx = {}
    if card_layout:
        resolved = _resolve_layout(card_layout)
        if resolved is not None:
            color_pairs, name_by_idx = resolved
            print(f"Using admin-supplied card layout with "
                  f"{len(set(color_pairs))} unique labels.")
        else:
            print("card_layout was provided but invalid; falling back to random.")

    if color_pairs is None:
        color_pairs, name_by_idx = _random_pairs()

    for row in range(grid_rows):
        for col in range(grid_cols):
            x = grid_start_x + col * card_spacing
            y = grid_start_y + row * card_spacing

            card_idx = row * grid_cols + col
            color_idx = color_pairs[card_idx]
            front_color = card_colors[color_idx]

            # Bottom layer: Colored card (fixed, stays in place)
            # The card body is buried inside the opaque slab so its sides and
            # bottom can't be seen. Its top face is lifted a hair (top_reveal)
            # ABOVE the slab surface (z=0) so it cleanly wins the depth test
            # instead of z-fighting with the coplanar slab top. Only this thin
            # top sliver is exposed, and the cover hides it until lifted.
            top_reveal = 0.001  # 1mm of the card's top above the floor
            colored_z = top_reveal - colored_thickness / 2
            colored_card = scene.add_entity(
                gs.morphs.Box(
                    size=(card_size, card_size, colored_thickness),
                    pos=(x, y, colored_z),
                    fixed=True,
                ),
                surface=gs.surfaces.Plastic(color=front_color),
            )

            # Top layer: Black cover (physics-enabled, can be picked up)
            # Sits on the card's exposed top, fully capping the color.
            cover_z = top_reveal + cover_thickness / 2
            cover = scene.add_entity(
                gs.morphs.Box(
                    size=(cover_size, cover_size, cover_thickness),
                    pos=(x, y, cover_z),
                    fixed=False,  # Can be picked up by robot!
                ),
                surface=gs.surfaces.Plastic(color=back_color),
            )

            cards.append({
                "colored_card": colored_card,
                "cover": cover,
                "row": row,
                "col": col,
                "pos": (x, y, cover_z),  # Cover position (now at cover_thickness/2) for robot targeting
                "color_idx": color_idx,
                "front_color": front_color,
                # Human-readable label (e.g. "dog") when an admin layout was
                # used; falls back to the generic color name otherwise.
                "label": name_by_idx.get(color_idx),
                "flipped": False,
                "matched": False,
            })

    # Build 2D grid reference for easy lookup
    card_grid = [[None for _ in range(grid_cols)] for _ in range(grid_rows)]
    for card in cards:
        card_grid[card["row"]][card["col"]] = card

    # Print layout info for debugging
    print(f"Grid: {grid_rows}x{grid_cols} = {num_cards} cards")
    print(f"Grid center: ({grid_center_x}, {grid_center_y})")
    print(f"Grid spans: x=[{grid_start_x - card_spacing/2:.2f}, {grid_start_x + (grid_cols-1)*card_spacing + card_spacing/2:.2f}]")
    print(f"Red robot at: {robot_red_pos}")
    print(f"Blue robot at: {robot_blue_pos}")

    # Print the world position (x, y, z) of every card. The stored "pos" is
    # the cover center, which is the point the robot targets when flipping.
    print("Card positions (x, y, z) -- z is the cover center / grab target:")
    for card in cards:
        cx, cy, cz = card["pos"]
        print(f"  card[row={card['row']}][col={card['col']}] "
              f"color_idx={card['color_idx']:2d}  "
              f"pos=({cx:.3f}, {cy:.3f}, {cz:.3f})")

    return {
        "robots": [robot_red, robot_blue],
        "objects": cards,
        "card_grid": card_grid,
        "grid_rows": grid_rows,
        "grid_cols": grid_cols,
        "turn_based": True,
        # Map of color_idx -> label name (empty when layout was random).
        "labels": name_by_idx,
    }
