import os
import base64
import tempfile
import numpy as np
from typing import List, Dict, Any, Optional

import genesis as gs

from .config import SCENES_DIR, BACKEND, SHOW_VIEWER

BACKEND_MAP = {
    "cpu": gs.cpu,
    "gpu": gs.gpu,
    "cuda": gs.cuda,
    "amdgpu": gs.amdgpu,
    "metal": gs.metal,
}

# Global flag for viewer - only one viewer can exist
_viewer_active = False


class GenesisSimulation:
    def __init__(self, scene_name: str, show_viewer: bool = None, card_layout=None):
        self.scene_name = scene_name
        self.scene = None
        self.robots = []
        self.objects = []
        self.camera = None
        self.recording = False
        self._initialized = False
        self._show_viewer = show_viewer if show_viewer is not None else SHOW_VIEWER
        # Card flip support
        self.card_grid = None
        self.grid_rows = 0
        self.grid_cols = 0
        self._pinned_covers = {}
        # Admin-supplied grid layout (2D names grid or dict); None -> random.
        self._card_layout = card_layout
        # Labels map (color_idx -> name) populated after the scene is built.
        self.labels = {}
        # Robot freeze support
        self._frozen_robots = {}
        # Store initial robot joint positions for reset_to_home
        self.robot_initial_qpos = []

    def build(self) -> None:
        global _viewer_active

        if self._initialized:
            return

        backend = BACKEND_MAP.get(BACKEND, gs.cpu)
        gs.init(backend=backend, theme="light")

        # Show viewer on server if enabled and no other viewer is active
        show_viewer = self._show_viewer and not _viewer_active

        # Adjust camera based on scene type
        if "competition" in self.scene_name:
            # View for competition grid with robots on opposite sides
            # Grid centered at x=0.55, robots at x~0.15 and x~0.95
            camera_pos = (0.55, -1.5, 1.2)  # Side view from negative Y
            camera_lookat = (0.55, 0.0, 0.1)  # Center of grid
            camera_fov = 55
        else:
            # Default side view
            camera_pos = (1.5, -1.0, 1.0)
            camera_lookat = (0.5, 0.0, 0.3)
            camera_fov = 45

        self.scene = gs.Scene(
            viewer_options=gs.options.ViewerOptions(
                camera_pos=camera_pos,
                camera_lookat=camera_lookat,
                camera_fov=camera_fov,
                max_FPS=60,
            ),
            sim_options=gs.options.SimOptions(dt=0.01),
            show_viewer=show_viewer,
        )

        if show_viewer:
            _viewer_active = True
            self._has_viewer = True
        else:
            self._has_viewer = False

        # Load scene setup function
        scene_result = self._load_scene(self.scene_name)

        # Add camera for recording with same position
        self.camera = self.scene.add_camera(
            res=(640, 480),
            pos=camera_pos,
            lookat=camera_lookat,
            fov=camera_fov,
            GUI=False,
        )

        self.scene.build()

        # Store robots from scene result
        if "robots" in scene_result:
            robots = scene_result["robots"]
            if isinstance(robots, list):
                self.robots = robots
            elif isinstance(robots, dict):
                self.robots = list(robots.values())

        self._setup_robot_control()

        # Store initial joint positions for each robot (for reset_to_home)
        self.robot_initial_qpos = []
        for robot in self.robots:
            qpos = robot.get_qpos()
            self.robot_initial_qpos.append(np.array(qpos.tolist()))

        self._initialized = True

    def _load_scene(self, scene_name: str):
        import os
        scene_file = os.path.join(SCENES_DIR, f"{scene_name}.py")
        if not os.path.exists(scene_file):
            plane = self.scene.add_entity(gs.morphs.Plane())
            franka = self.scene.add_entity(
                gs.morphs.MJCF(file="xml/franka_emika_panda/panda.xml")
            )
            return {"robots": [franka]}

        import importlib.util
        spec = importlib.util.spec_from_file_location(scene_name, scene_file)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        # >>> the fix: forward the admin layout into setup() <<<
        result = module.setup(self.scene, card_layout=self._card_layout)

        if "card_grid" in result:
            self.card_grid = result["card_grid"]
            self.grid_rows = result.get("grid_rows", 5)
            self.grid_cols = result.get("grid_cols", 6)
        # Capture color_idx -> label name map (empty for random layouts).
        self.labels = result.get("labels", {})

        return result

    def _setup_robot_control(self) -> None:
        for robot in self.robots:
            # Set PD control gains for Franka
            robot.set_dofs_kp(
                np.array([4500, 4500, 3500, 3500, 2000, 2000, 2000, 100, 100])
            )
            robot.set_dofs_kv(
                np.array([450, 450, 350, 350, 200, 200, 200, 10, 10])
            )
            robot.set_dofs_force_range(
                np.array([-87, -87, -87, -87, -12, -12, -12, -100, -100]),
                np.array([87, 87, 87, 87, 12, 12, 12, 100, 100]),
            )

    def _move_to_ready_position(self) -> None:
        """Move robots to a safe ready position after scene build.

        Uses motion planning for smooth movement that won't swing low.
        """
        for robot in self.robots:
            end_effector = robot.get_link("hand")
            quat = np.array([0, 1, 0, 0])  # Gripper down

            # Safe ready position: high and near robot base
            ready_pos = np.array([0.3, 0.0, 0.55])

            qpos_goal = robot.inverse_kinematics(
                link=end_effector,
                pos=ready_pos,
                quat=quat,
            )

            # Use motion planning for smooth movement
            try:
                path = robot.plan_path(
                    qpos_goal=qpos_goal,
                    num_waypoints=100,
                )
                for waypoint in path:
                    robot.control_dofs_position(waypoint[:-2], np.arange(7))
                    robot.control_dofs_position(np.array([0.04, 0.04]), np.arange(7, 9))
                    self.scene.step()
            except Exception:
                # Fallback: just set position and wait
                robot.control_dofs_position(qpos_goal[:-2], np.arange(7))
                robot.control_dofs_position(np.array([0.04, 0.04]), np.arange(7, 9))
                for _ in range(100):
                    self.scene.step()

        # Settling time
        for _ in range(30):
            self.scene.step()

    def add_object(
        self,
        obj_type: str,
        position: List[float],
        size: Optional[List[float]] = None,
        radius: Optional[float] = None,
    ) -> int:
        if obj_type == "cube":
            entity = self.scene.add_entity(
                gs.morphs.Box(size=tuple(size), pos=tuple(position), fixed=False)
            )
        elif obj_type == "sphere":
            entity = self.scene.add_entity(
                gs.morphs.Sphere(radius=radius, pos=tuple(position), fixed=False)
            )
        elif obj_type == "cylinder":
            entity = self.scene.add_entity(
                gs.morphs.Cylinder(
                    height=size[2] if size else 0.1,
                    radius=radius or 0.05,
                    pos=tuple(position),
                    fixed=False,
                )
            )
        else:
            raise ValueError(f"Unknown object type: {obj_type}")

        obj_id = len(self.objects)
        self.objects.append({
            "id": obj_id,
            "type": obj_type,
            "entity": entity,
        })
        return obj_id

    def move_robot(self, robot_id: int, position: List[float]) -> List[float]:
        """Move robot to position (sets target, doesn't wait)."""
        robot = self.robots[robot_id]
        end_effector = robot.get_link("hand")

        # End-effector orientation: gripper pointing downwards
        # IK works in world frame, so same quaternion for both robots
        quat = np.array([0, 1, 0, 0])

        qpos = robot.inverse_kinematics(
            link=end_effector,
            pos=np.array(position),
            quat=quat,
        )

        robot.control_dofs_position(qpos[:-2], np.arange(7))
        return qpos.tolist()

    def move_robot_smooth(self, robot_id: int, position: List[float], num_waypoints: int = 50) -> None:
        """Move robot smoothly using motion planning.

        Uses plan_path() to interpolate waypoints for smooth trajectory.
        For safety, first lifts to a safe height if current position is low.
        """
        robot = self.robots[robot_id]
        end_effector = robot.get_link("hand")
        quat = np.array([0, 1, 0, 0])  # Gripper down

        # Get current end effector position
        current_pos = end_effector.get_pos()
        current_z = float(current_pos[2])
        target_z = position[2]

        # Safe height to avoid collisions
        safe_z = 0.5

        # If starting low, first lift straight up to safe height
        if current_z < safe_z - 0.05:
            lift_pos = np.array([float(current_pos[0]), float(current_pos[1]), safe_z])
            qpos_lift = robot.inverse_kinematics(link=end_effector, pos=lift_pos, quat=quat)
            try:
                path = robot.plan_path(qpos_goal=qpos_lift, num_waypoints=200)
                for waypoint in path:
                    robot.control_dofs_position(waypoint[:-2], np.arange(7))
                    self.step(1)
            except Exception:
                robot.control_dofs_position(qpos_lift[:-2], np.arange(7))
                self.step(60)

        # Now move to target position
        qpos_goal = robot.inverse_kinematics(
            link=end_effector,
            pos=np.array(position),
            quat=quat,
        )

        try:
            path = robot.plan_path(
                qpos_goal=qpos_goal,
                num_waypoints=num_waypoints,
            )

            for waypoint in path:
                robot.control_dofs_position(waypoint[:-2], np.arange(7))
                self.step(1)

            self.step(15)
        except Exception as e:
            print(f"Motion planning failed for robot {robot_id} to {position}: {e}")
            robot.control_dofs_position(qpos_goal[:-2], np.arange(7))
            self.step(num_waypoints + 20)

    def move_joints(self, robot_id: int, angles: List[float]) -> None:
        robot = self.robots[robot_id]
        robot.control_dofs_position(np.array(angles), np.arange(len(angles)))

    def reset_gripper(self, robot_id: int, open_width: float = 0.04) -> None:
        """Force a robot's gripper fingers back to a clean OPEN pose.

        Uses set_dofs_position (hard kinematic reset) rather than control_dofs_*
        (a target the PD controller chases), so it recovers fingers that are
        jammed, splayed, or past their limits.
        """
        robot = self.robots[robot_id]
        fingers_dof = np.arange(7, 9)
        finger_pos = np.array([open_width, open_width])

        # 1. Clear any persistent closing force first.
        robot.control_dofs_force(np.array([0.0, 0.0]), fingers_dof)

        # 2. Hard-set the finger joint positions (immediate, bypasses PD).
        try:
            robot.set_dofs_position(finger_pos, fingers_dof)
        except TypeError:
            # Some Genesis versions expect the local-dofs kwarg name.
            robot.set_dofs_position(position=finger_pos, dofs_idx_local=fingers_dof)

        # 3. Zero finger velocity so they don't spring away from the reset pose.
        try:
            robot.set_dofs_velocity(np.array([0.0, 0.0]), fingers_dof)
        except Exception:
            try:
                robot.zero_all_dofs_velocity()
            except AttributeError:
                pass

        # 4. Re-assert a position hold so the PD controller keeps them open.
        robot.control_dofs_position(finger_pos, fingers_dof)

        # Let the reset settle.
        self.step(20)

    def gripper(self, robot_id: int, action: str) -> None:
        robot = self.robots[robot_id]
        fingers_dof = np.arange(7, 9)

        if action == "open":
            print(f"Open Gripper")
            # IMPORTANT: closing uses FORCE control, which persists every sim step.
            # Simply commanding a position doesn't reliably cancel that residual
            # closing force -- with weak finger gains (kp=100) the leftover force
            # keeps the fingers clamped, so the gripper "stays closed". Explicitly
            # zero the force FIRST, then command the open position so the PD
            # controller is actually free to drive the fingers apart.
            robot.control_dofs_force(np.array([0.0, 0.0]), fingers_dof)
            robot.control_dofs_position(np.array([0.04, 0.04]), fingers_dof)
        elif action == "close":
            print(f"Close Gripper")
            # The reference uses -0.5 N and it holds -- because it never carries the
            # cube far. We transport across the grid, so we need a firmer hold.
            # TUNING KNOB: reduce if the box is crushed/ejected; increase if it slips.
            robot.control_dofs_force(np.array([-4.0, -4.0]), fingers_dof)

    def get_state(self, robot_id: int) -> Dict[str, Any]:
        robot = self.robots[robot_id]
        qpos = robot.get_qpos()
        end_effector = robot.get_link("hand")
        ee_pos = end_effector.get_pos()

        return {
            "joints": qpos.tolist(),
            "end_effector": {
                "position": ee_pos.tolist(),
            },
            "collision": False,
        }
    # ===========================================================================
# COVER PINNING -- prevents the arm from knocking neighboring covers.
#
# The covers are dynamic (fixed=False) only so the target one can be picked
# up. While a flip is in progress we "pin" every OTHER cover: each sim step
# we snap it back to its captured pose and zero its velocity, so a stray
# brush from the arm's elbow/forearm can't move it. This mirrors how step()
# already re-asserts frozen robot positions.
#
# Requires an attribute on GenesisSimulation. Add to __init__:
#     self._pinned_covers = {}
# (a dict of id(entity) -> (entity, pos_np, quat_np))
# ===========================================================================
    def pin_cover(self, cover_entity) -> None:
        """Lock a single cover entity at its current pose."""
        pos = cover_entity.get_pos()
        quat = cover_entity.get_quat()
        self._pinned_covers[id(cover_entity)] = (
            cover_entity,
            np.array(pos.tolist()),
            np.array(quat.tolist()),
        )


    def unpin_cover(self, cover_entity) -> None:
        """Release a single cover so it can move freely (e.g. to be picked up)."""
        self._pinned_covers.pop(id(cover_entity), None)


    def pin_all_covers(self, exclude=None) -> None:
        """Pin every not-yet-flipped cover, optionally excluding one card.

        Pass the card currently being flipped as `exclude` if you don't want
        its cover pinned; here we pin it too during the approach and only
        release it right before the grasp.
        """
        self._pinned_covers = {}
        if self.card_grid is None:
            return
        for row in self.card_grid:
            for card in row:
                if card is None or card.get("flipped"):
                    continue
                if exclude is not None and card is exclude:
                    continue
                self.pin_cover(card["cover"])

    def unpin_all_covers(self) -> None:
        """Release all pinned covers."""
        self._pinned_covers = {}

    def step(self, steps: int) -> None:
        for _ in range(steps):
            # Re-apply frozen robot positions before each step to prevent drift
            for robot_id, frozen_pos in self._frozen_robots.items():
                if robot_id < len(self.robots):
                    robot = self.robots[robot_id]
                    robot.control_dofs_position(frozen_pos[:7], np.arange(7))
                    robot.control_dofs_position(frozen_pos[7:9], np.arange(7, 9))

            # Hold every pinned cover locked at its captured pose so the arm
            # can't knock it. set_pos/set_quat + zeroing velocity kinematically
            # freezes the free body.
            for entity, pos, quat in self._pinned_covers.values():
                entity.set_pos(pos)
                entity.set_quat(quat)
                try:
                    entity.zero_all_dofs_velocity()
                except AttributeError:
                    pass  # older Genesis: pose reset alone still holds it

            self.scene.step()
            if self.recording and self.camera:
                self.camera.render()

    def freeze_robot(self, robot_id: int) -> None:
        """Freeze a robot at its current position.

        The robot will maintain this position during simulation steps.
        """
        if robot_id >= len(self.robots):
            return

        robot = self.robots[robot_id]
        qpos = robot.get_qpos()
        # Convert Genesis tensor to numpy array
        self._frozen_robots[robot_id] = np.array(qpos.tolist())

    def unfreeze_robot(self, robot_id: int) -> None:
        """Unfreeze a robot, allowing it to move freely."""
        if robot_id in self._frozen_robots:
            del self._frozen_robots[robot_id]

    def is_robot_frozen(self, robot_id: int) -> bool:
        """Check if a robot is frozen."""
        return robot_id in self._frozen_robots

    def reset_to_home(self, robot_id: int) -> None:
        """Reset robot to its initial joint configuration.

        This restores the robot to the exact pose it had when the scene was built,
        which is more reliable than computing IK for a home position.
        """
        if robot_id >= len(self.robots) or robot_id >= len(self.robot_initial_qpos):
            return

        robot = self.robots[robot_id]
        initial_qpos = self.robot_initial_qpos[robot_id]

        # Control arm joints (0-6) to initial positions
        robot.control_dofs_position(initial_qpos[:7], np.arange(7))
        # Open gripper
        robot.control_dofs_position(np.array([0.04, 0.04]), np.arange(7, 9))
        self.step(150)  # More time to settle

    def reset(self) -> None:
        self.scene.reset()
    
    def destroy(self) -> None:
        """Tear down this simulation and release Genesis' global state.

        Safe to call multiple times and even if build() never ran. Note that
        gs.destroy() tears down Genesis process-wide, so this must not run
        while another GenesisSimulation is still active in the same process.
        """
        global _viewer_active

        if not self._initialized:
            # Nothing was built; still make sure we don't leak the viewer flag.
            if getattr(self, "_has_viewer", False):
                _viewer_active = False
                self._has_viewer = False
            return

        # Stop any in-progress recording so the camera isn't left mid-capture.
        if self.recording and self.camera:
            try:
                self.camera.stop_recording()
            except Exception as exc:
                print(f"Warning: failed to stop recording during destroy: {exc!r}")
            finally:
                self.recording = False

        # Release the global viewer claim so future sims can open a viewer.
        if getattr(self, "_has_viewer", False):
            _viewer_active = False
            self._has_viewer = False

        # Tear down Genesis' backend/GPU state (counterpart to gs.init()).
        try:
            gs.destroy()
        except Exception as exc:
            print(f"Warning: gs.destroy() failed during teardown: {exc!r}")

        # Drop references so Python can reclaim the rest.
        self.scene = None
        self.camera = None
        self.robots = []
        self.objects = []
        self._frozen_robots = {}
        self.card_grid = None
        self._initialized = False

    def start_recording(self) -> None:
        if self.camera:
            self.camera.start_recording()
            self.recording = True

    def stop_recording(self) -> bytes:
        if not self.camera or not self.recording:
            return b""

        self.recording = False

        with tempfile.NamedTemporaryFile(suffix=".mp4", delete=False) as f:
            temp_path = f.name

        self.camera.stop_recording(save_to_filename=temp_path, fps=60)

        # Try to re-encode to browser-compatible H.264
        output_path = self._reencode_video(temp_path)

        with open(output_path, "rb") as f:
            video_data = f.read()

        os.unlink(output_path)
        if output_path != temp_path and os.path.exists(temp_path):
            os.unlink(temp_path)

        return video_data

    def _reencode_video(self, input_path: str) -> str:
        """Re-encode video to H.264 for browser compatibility."""
        import subprocess
        import shutil

        # Try ffmpeg first
        if shutil.which("ffmpeg"):
            output_path = input_path.replace(".mp4", "_h264.mp4")
            try:
                subprocess.run([
                    "ffmpeg", "-y", "-i", input_path,
                    "-c:v", "libx264", "-preset", "fast",
                    "-movflags", "+faststart",
                    "-pix_fmt", "yuv420p",
                    output_path
                ], capture_output=True, check=True)
                return output_path
            except subprocess.CalledProcessError:
                pass

        # Try imageio-ffmpeg (bundled ffmpeg)
        try:
            import imageio_ffmpeg
            ffmpeg_path = imageio_ffmpeg.get_ffmpeg_exe()
            output_path = input_path.replace(".mp4", "_h264.mp4")
            subprocess.run([
                ffmpeg_path, "-y", "-i", input_path,
                "-c:v", "libx264", "-preset", "fast",
                "-movflags", "+faststart",
                "-pix_fmt", "yuv420p",
                output_path
            ], capture_output=True, check=True)
            return output_path
        except (ImportError, subprocess.CalledProcessError):
            pass

        # Return original if re-encoding fails
        return input_path

    def get_objects(self) -> List[Dict[str, Any]]:
        result = []
        for obj in self.objects:
            entity = obj["entity"]
            pos = entity.get_pos()
            quat = entity.get_quat()
            result.append({
                "id": obj["id"],
                "type": obj["type"],
                "position": pos.tolist(),
                "orientation": quat.tolist(),
            })
        return result

    # Card flip methods for competition scenes

    def point_at_card(self, row: int, col: int, robot_id: int = 0) -> Dict[str, Any]:
        """Move robot to point at a card.

        Args:
            row: Grid row (0 to grid_rows-1)
            col: Grid column (0 to grid_cols-1)
            robot_id: Which robot to move (0=red, 1=blue)

        Returns:
            Dict with card position info
        """
        if self.card_grid is None:
            raise ValueError("No card grid in this scene")

        if row < 0 or row >= self.grid_rows:
            raise ValueError(f"Row {row} out of range (0-{self.grid_rows-1})")
        if col < 0 or col >= self.grid_cols:
            raise ValueError(f"Col {col} out of range (0-{self.grid_cols-1})")

        card = self.card_grid[row][col]
        card_pos = card["pos"]
        hover_height = 0.25  # Hover above the card

        # Move robot end-effector above the card
        target_pos = [card_pos[0], card_pos[1], card_pos[2] + hover_height]
        self.move_robot(robot_id, target_pos)
        self.step(60)  # Let robot move to position

        return {
            "row": row,
            "col": col,
            "card_pos": card_pos,
            "robot_id": robot_id,
        }

    def flip_card(self, row: int, col: int, robot_id: int = 0) -> dict:
        if self.card_grid is None:
            raise ValueError("No card grid in this scene")
        if row < 0 or row >= self.grid_rows:
            raise ValueError(f"Row {row} out of range (0-{self.grid_rows-1})")
        if col < 0 or col >= self.grid_cols:
            raise ValueError(f"Col {col} out of range (0-{self.grid_cols-1})")

        card = self.card_grid[row][col]

        if card["flipped"]:
            return {
                "color_idx": card["color_idx"],
                "already_flipped": True,
                "matched": card.get("matched", False),
            }

        # Ensure the gripper starts OPEN (clears any residual close force).
        self.reset_gripper(robot_id)
        

        # Pin all not-yet-flipped covers so the approach can't knock neighbors.
        self.pin_all_covers()

        cover_x, cover_y, cover_z = card["pos"]
        cover_entity = card["cover"]

        print(f"Robot {robot_id} flipping card at row={row}, col={col}, "
            f"world pos=({cover_x:.3f}, {cover_y:.3f}, {cover_z:.3f})")

        # --- Heights (unchanged) ---
        safe_z = 0.35
        hover_z = cover_z + 0.23
        grab_z = cover_z + 0.11
        lift_z = cover_z + 0.26
        place_z = cover_z + 0.09

        if robot_id == 0:
            discard_x, discard_y = 0.00, 1.10
        else:
            discard_x, discard_y = 1.40, -1.15

        discard_z = cover_z

        # === APPROACH (faster travel) ===
        self.move_robot_smooth(robot_id, [cover_x, cover_y, hover_z], num_waypoints=100)
        self.step(30)
        self.gripper(robot_id, "open")
        self.step(30)

        # === DESCEND STRAIGHT DOWN ===
        self.move_robot(robot_id, [cover_x, cover_y, grab_z])
        self.step(75)

        # === GRASP (hold + close). Keep enough settle for a firm grip. ===
        self.move_robot(robot_id, [cover_x, cover_y, grab_z])  # re-assert hold
        self.gripper(robot_id, "close")
        self.step(90)

        # Release only the target cover so it can be lifted.
        self.unpin_cover(card["cover"])

        # === LIFT STRAIGHT UP ===
        #self.move_robot(robot_id, [cover_x, cover_y, lift_z])
        #self.step(40)
        self.move_robot(robot_id, [cover_x, cover_y, safe_z])
        self.step(10)

            # === TELEPORT the cover from the gripper to the discard pile ===
        # Open the gripper first so the fingers release, then snap the cover to the
        # discard pile and let it settle there. No transport/place motion needed.
        #self.gripper(robot_id, "open")
        #self.step(20)

        cover_entity.set_pos(np.array([discard_x, discard_y, discard_z]))
        cover_entity.set_quat(np.array([1.0, 0.0, 0.0, 0.0]))
        try:
            cover_entity.zero_all_dofs_velocity()
        except AttributeError:
            pass

        # Briefly pin the cover at the pile so it doesn't get nudged while the
        # sim settles, then release it to normal physics.
        self.pin_cover(cover_entity)
        self.step(20)
        self.unpin_cover(cover_entity)




        # === RETURN HOME ===
        self.reset_to_home(robot_id)

        # Release all pinned covers.
        self.unpin_all_covers()

        card["flipped"] = True

        return {
            "color_idx": card["color_idx"],
            "already_flipped": False,
            "matched": card.get("matched", False),
        }


    def unflip_card(self, row: int, col: int) -> None:
        """Flip a card back to face-down by TELEPORTING its cover back on top.

        The cover entity still exists (it was carried to the discard pile by
        flip_card). We snap that same entity back to the card's original cover
        pose and zero its velocity, so the spot is physically re-covered. This
        reuses the same set_pos/set_quat mechanism as cover pinning.
        """
        if self.card_grid is None:
            return

        card = self.card_grid[row][col]
        if card["flipped"] and not card.get("matched", False):
            card["flipped"] = False

            cover = card["cover"]
            x, y, cover_z = card["pos"]  # original cover center, stored at build
            cover.set_pos(np.array([x, y, cover_z]))
            cover.set_quat(np.array([1.0, 0.0, 0.0, 0.0]))  # identity orientation
            try:
                cover.zero_all_dofs_velocity()
            except AttributeError:
                pass  # older Genesis: pose reset alone still places it

            # Briefly pin the cover so it can't drift while the sim settles it
            # back onto the card top.
            self.pin_cover(cover)
            self.step(30)
            self.unpin_cover(cover)

    def cover_card(self, row: int, col: int) -> bool:
        """Force the cover back onto a single card, regardless of its state.

        Returns True if a card existed at (row, col) and was re-covered.
        Teleports the existing cover entity back to its original pose (same
        set_pos/pin mechanism as unflip_card) and clears flipped/matched.
        """
        import numpy as np
        if self.card_grid is None:
            return False
        if row < 0 or row >= self.grid_rows or col < 0 or col >= self.grid_cols:
            raise ValueError(f"({row},{col}) out of range for "
                            f"{self.grid_rows}x{self.grid_cols} grid")

        card = self.card_grid[row][col]
        if card is None:
            return False

        # Clear logical state -- force face-down even if it was a matched pair.
        card["flipped"] = False
        card["matched"] = False

        cover = card["cover"]
        x, y, cover_z = card["pos"]  # original cover center, stored at build
        cover.set_pos(np.array([x, y, cover_z]))
        cover.set_quat(np.array([1.0, 0.0, 0.0, 0.0]))
        try:
            cover.zero_all_dofs_velocity()
        except AttributeError:
            pass

        # Briefly pin so it settles cleanly onto the card top.
        self.pin_cover(cover)
        self.step(30)
        self.unpin_cover(cover)
        return True

    def reset_board(self) -> None:
        """Re-cover the ENTIRE board: teleport every cover back to its spot and
        clear each card's flipped/matched flags. Instant, no arm motion.

        Reuses the same set_pos/set_quat + brief-pin approach as unflip_card, but
        applies it to every card regardless of flipped/matched state (a matched
        pair's covers were discarded too, so they must come back on a full reset).
        """
        import numpy as np
        if self.card_grid is None:
            return

        covers_to_settle = []
        for row in self.card_grid:
            for card in row:
                if card is None:
                    continue
                cover = card["cover"]
                x, y, cover_z = card["pos"]  # original cover center from build
                cover.set_pos(np.array([x, y, cover_z]))
                cover.set_quat(np.array([1.0, 0.0, 0.0, 0.0]))
                try:
                    cover.zero_all_dofs_velocity()
                except AttributeError:
                    pass
                # Reset logical state so the game treats them as fresh face-down.
                card["flipped"] = False
                card["matched"] = False
                # Pin so it can't drift while the sim settles.
                self.pin_cover(cover)
                covers_to_settle.append(cover)

        # Let all covers settle onto their card tops at once, then release pins.
        self.step(30)
        for cover in covers_to_settle:
            self.unpin_cover(cover)

    def get_card_state(self, row: int, col: int) -> Dict[str, Any]:
        """Get state of a specific card."""
        if self.card_grid is None:
            raise ValueError("No card grid in this scene")

        if row < 0 or row >= self.grid_rows:
            raise ValueError(f"Row {row} out of range")
        if col < 0 or col >= self.grid_cols:
            raise ValueError(f"Col {col} out of range")

        card = self.card_grid[row][col]
        return {
            "row": row,
            "col": col,
            "flipped": card["flipped"],
            "matched": card.get("matched", False),
            "color_idx": card["color_idx"] if card["flipped"] else None,
        }

    def get_grid_state(self) -> List[List[Dict[str, Any]]]:
        """Get state of all cards in the grid."""
        if self.card_grid is None:
            raise ValueError("No card grid in this scene")

        result = []
        for row in range(self.grid_rows):
            row_state = []
            for col in range(self.grid_cols):
                row_state.append(self.get_card_state(row, col))
            result.append(row_state)
        return result
