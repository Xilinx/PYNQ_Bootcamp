import json
import base64
import tempfile
from typing import List, Dict, Any, Optional

import requests


class SimulationClient:
    """Client for Genesis remote simulation server.

    This client allows you to control a robot arm running on a remote server.
    Use create_environment() for standard mode (your own isolated simulation)
    or join_competition() for competition mode (shared scene with other teams).
    """

    def __init__(self, server_ip: str, port: int = 9002):
        """Connect to a Genesis simulation server.

        Args:
            server_ip: IP address of the server (ask your instructor!)
            port: Server port (default: 9002)
        """
        self.server_url = f"http://{server_ip}:{port}"
        self.token: Optional[str] = None
        self._competition_mode = False

    def _request(self, action: str, params: Dict[str, Any] = None) -> Dict[str, Any]:
        """Send a request to the server."""
        payload = {
            "action": action,
            "token": self.token,
            "params": params or {},
        }

        response = requests.post(self.server_url, json=payload)
        data = response.json()

        if data.get("status") == "error":
            raise RuntimeError(data.get("message", "Unknown error"))

        return data

    # Standard Mode Methods

    def create_environment(self, scene: str = "empty") -> None:
        """Create a new simulation environment.

        Args:
            scene: Scene to load. Options: "empty", "pick_and_place"
        """
        result = self._request("create_env", {"scene": scene})
        self.token = result["token"]
        self._competition_mode = False

    def add_cube(
        self, position: List[float], size: List[float] = None
    ) -> int:
        """Add a cube to the scene.

        Args:
            position: [x, y, z] position in meters
            size: [width, height, depth] in meters (default: [0.04, 0.04, 0.04])

        Returns:
            Object ID for the new cube
        """
        if size is None:
            size = [0.04, 0.04, 0.04]
        result = self._request("add_object", {
            "type": "cube",
            "position": position,
            "size": size,
        })
        return result["object_id"]

    def add_sphere(self, position: List[float], radius: float = 0.03) -> int:
        """Add a sphere to the scene.

        Args:
            position: [x, y, z] position in meters
            radius: Radius in meters (default: 0.03)

        Returns:
            Object ID for the new sphere
        """
        result = self._request("add_object", {
            "type": "sphere",
            "position": position,
            "radius": radius,
        })
        return result["object_id"]

    def add_cylinder(
        self, position: List[float], radius: float = 0.03, height: float = 0.1
    ) -> int:
        """Add a cylinder to the scene.

        Args:
            position: [x, y, z] position in meters
            radius: Radius in meters (default: 0.03)
            height: Height in meters (default: 0.1)

        Returns:
            Object ID for the new cylinder
        """
        result = self._request("add_object", {
            "type": "cylinder",
            "position": position,
            "radius": radius,
            "size": [0, 0, height],
        })
        return result["object_id"]

    def move_robot(
        self, robot_id: int = 0, position: List[float] = None,
        smooth: bool = False, num_waypoints: int = 50
    ) -> List[float]:
        """Move the robot end effector to a position.

        Uses inverse kinematics to calculate joint angles.

        Args:
            robot_id: Which robot (default: 0)
            position: [x, y, z] target position in meters
            smooth: If True, use motion planning for smooth trajectory
            num_waypoints: Number of waypoints for smooth motion (default: 50)

        Returns:
            Joint angles used to reach the position (empty if smooth=True)
        """
        params = {"position": position, "smooth": smooth}
        if smooth:
            params["num_waypoints"] = num_waypoints
        if not self._competition_mode:
            params["robot_id"] = robot_id
        result = self._request("move_robot", params)
        return result.get("joint_angles", [])

    def move_joints(self, robot_id: int, angles: List[float]) -> None:
        """Move robot joints directly.

        Args:
            robot_id: Which robot
            angles: List of joint angles in radians
        """
        self._request("move_joints", {"robot_id": robot_id, "angles": angles})

    def open_gripper(self, robot_id: int = 0) -> None:
        """Open the robot gripper.

        Args:
            robot_id: Which robot (default: 0)
        """
        params = {"action": "open"}
        if not self._competition_mode:
            params["robot_id"] = robot_id
        self._request("gripper", params)

    def close_gripper(self, robot_id: int = 0) -> None:
        """Close the robot gripper.

        Args:
            robot_id: Which robot (default: 0)
        """
        params = {"action": "close"}
        if not self._competition_mode:
            params["robot_id"] = robot_id
        self._request("gripper", params)

    def get_state(self, robot_id: int = 0) -> Dict[str, Any]:
        """Get the current robot state.

        Args:
            robot_id: Which robot (default: 0)

        Returns:
            Dict with 'joints' and 'end_effector' position
        """
        params = {} if self._competition_mode else {"robot_id": robot_id}
        return self._request("get_state", params)

    def step(self, steps: int = 1) -> None:
        """Step the simulation forward.

        Note: In competition mode, the server auto-steps so this is not needed.

        Args:
            steps: Number of simulation steps (default: 1)
        """
        if self._competition_mode:
            raise RuntimeError("Cannot call step() in competition mode - server auto-steps")
        self._request("step", {"steps": steps})

    def reset(self) -> None:
        """Reset the simulation to its initial state."""
        self._request("reset")

    def reset_robot(self, robot_id: int = 0) -> None:
        """Reset a robot to its initial/home position.

        Args:
            robot_id: Which robot to reset (default: 0)
        """
        self._request("reset_robot", {"robot_id": robot_id})

    def start_recording(self) -> None:
        """Start recording video of the simulation."""
        self._request("start_recording")

    def stop_recording(self) -> bytes:
        """Stop recording and get the video data.

        Returns:
            Video data as bytes (MP4 format)
        """
        result = self._request("stop_recording")
        video_base64 = result.get("video_base64", "")
        return base64.b64decode(video_base64)

    def show_video(self, video_data: bytes) -> None:
        """Display video in Jupyter notebook.

        Args:
            video_data: Video bytes from stop_recording()
        """
        try:
            from IPython.display import HTML, display

            # Embed video as base64 data URI for browser compatibility
            video_b64 = base64.b64encode(video_data).decode('utf-8')
            html = f'''
            <video controls width="640">
                <source src="data:video/mp4;base64,{video_b64}" type="video/mp4">
                Your browser does not support the video tag.
            </video>
            '''
            display(HTML(html))
        except ImportError:
            print("IPython not available.")
            # Save to file as fallback
            with tempfile.NamedTemporaryFile(suffix=".mp4", delete=False) as f:
                f.write(video_data)
                print(f"Video saved to: {f.name}")

    def get_objects(self) -> List[Dict[str, Any]]:
        """Get all objects in the scene.

        Returns:
            List of object dicts with 'id', 'type', 'position', 'orientation'
        """
        result = self._request("get_objects")
        return result.get("objects", [])

    def destroy(self) -> None:
        """Destroy your simulation environment.

        Always call this when you're done!
        """
        if self._competition_mode:
            self.leave_competition()
        else:
            self._request("destroy_env")
        self.token = None

    # Competition Mode Methods

    def join_competition(self, team_id, password=None):
        """Join an active competition.

        Args:
            team_id: Your team name (e.g., "team_red" or "team_blue")
            password: The join password the instructor set for your team
                      (omit if the instructor set no password for it)
        """
        result = self._request("join_competition", {
            "team_id": team_id,
            "password": password,
        })
        self.token = result["token"]
        self._competition_mode = True

    def leave_competition(self) -> None:
        """Leave the competition."""
        self._request("leave_competition")
        self.token = None
        self._competition_mode = False

    def get_competition_state(self) -> Dict[str, Any]:
        """Get the current competition state.

        Returns:
            Dict with 'robots', 'objects', 'current_turn', 'scores'
        """
        return self._request("get_competition_state")

    def flip_card(self, row: int, col: int) -> Dict[str, Any]:
        """Flip a card at the given grid position.

        This is the main action in competition mode. Flip two cards
        per turn - if they match, you score a point!

        Args:
            row: Row (0-3, bottom to top)
            col: Column (0-3, left to right)

        Returns:
            Dict with:
            - color_idx: The revealed color (0-7)
            - already_flipped: True if card was already face-up
            - match_result: If second flip, contains match info
        """
        return self._request("flip_card", {"row": row, "col": col})

    def get_card_state(self, row: int, col: int) -> Dict[str, Any]:
        """Get state of a specific card.

        Args:
            row: Row (0-3)
            col: Column (0-3)

        Returns:
            Dict with flipped, matched, and color_idx (if flipped)
        """
        return self._request("get_card_state", {"row": row, "col": col})

    def get_grid_state(self) -> List[List[Dict[str, Any]]]:
        """Get state of all cards on the grid.

        Returns:
            2D list of card states. Each card has:
            - row, col: Position
            - flipped: True if face-up
            - matched: True if part of a matched pair
            - color_idx: Color index (only visible if flipped)
        """
        result = self._request("get_grid_state")
        return result.get("grid", [])

    def end_turn(self) -> str:
        """End your turn (in turn-based competitions).

        Returns:
            The team ID of who plays next
        """
        result = self._request("end_turn")
        return result.get("next_turn", "")

    # Admin Methods (for instructors)

    def admin_start_competition(self, scene, password, card_layout=None,
                                join_passwords=None):
        """Start a competition (instructor only).

        Args:
            scene: Competition scene name
            password: Admin password
            card_layout: Optional custom card layout
            join_passwords: Optional dict {team_id: join_password}. Teams omitted
                            here can join without a password.
        """
        self._request("admin_start_competition", {
            "scene": scene,
            "password": password,
            "card_layout": card_layout,
            "join_passwords": join_passwords,
        })

    def admin_stop_competition(self, password: str) -> None:
        """Stop the competition (instructor only).

        Args:
            password: Admin password
        """
        self._request("admin_stop_competition", {"password": password})
        
    def admin_reset_board(self, password: str) -> None:
        """Reset the board (instructor only): re-cover all cards and zero scores.

        Keeps the same layout and does NOT tear down the competition -- use this
        when you just want a clean board mid-session instead of restarting.

        Args:
            password: Admin password
        """
        self._request("admin_reset_board", {"password": password})

    def admin_get_status(self, password: str) -> Dict[str, Any]:
        """Get server status (instructor only).

        Args:
            password: Admin password

        Returns:
            Dict with 'sessions' count and 'competition_active' flag
        """
        return self._request("admin_get_status", {"password": password})

    def admin_list_card_images(self, password: str) -> List[str]:
        """List available card images (instructor only).

        Args:
            password: Admin password

        Returns:
            List of image filenames
        """
        result = self._request("admin_list_card_images", {"password": password})
        return result.get("images", [])

    def admin_set_card_layout(
        self, password: str, layout: Dict[str, Any]
    ) -> None:
        """Set custom card layout (instructor only).

        Args:
            password: Admin password
            layout: Card layout with 'cards' list containing row/col/image
        """
        self._request("admin_set_card_layout", {
            "password": password,
            "layout": layout,
        })
