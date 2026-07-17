import json
import base64
from http.server import HTTPServer, BaseHTTPRequestHandler
from typing import Dict, Any, Optional


from .config import PORT, MAX_SESSIONS, SESSION_TIMEOUT, CLEANUP_INTERVAL, ADMIN_PASSWORD, CARD_IMAGES_DIR
from .session_manager import SessionManager
from .simulation import GenesisSimulation
from .competition import CompetitionManager
import os


class GenesisRequestHandler(BaseHTTPRequestHandler):
    session_manager: SessionManager = None
    simulations: Dict[str, GenesisSimulation] = {}
    competition_manager: CompetitionManager = None

    def log_message(self, format, *args):
        """Override to show client connections clearly."""
        client_ip = self.client_address[0]
        print(f"\r  [{client_ip}] {args[0]:<50}", flush=True)

    def do_POST(self):
        content_length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(content_length).decode("utf-8")

        try:
            request = json.loads(body)
        except json.JSONDecodeError:
            self._send_error("Invalid JSON")
            return

        action = request.get("action")
        token = request.get("token")
        params = request.get("params", {})

        client_ip = self.client_address[0]
        print(f"\r  [{client_ip}] action={action:<25}", flush=True)

        try:
            result = self._handle_action(action, token, params)
            self._send_response(result)
        except Exception as e:
            self._send_error(str(e))

    def _handle_action(
        self, action: str, token: Optional[str], params: Dict[str, Any]
    ) -> Dict[str, Any]:
        # Actions that don't require a token
        if action == "create_env":
            return self._create_env(params)

        # Competition actions that don't require standard token
        if action == "join_competition":
            return self._join_competition(params)

        if action == "get_competition_state":
            return self.competition_manager.get_state()

        # Admin actions
        if action and action.startswith("admin_"):
            return self._handle_admin_action(action, params)

        # All other actions require a valid token
        if not token:
            raise ValueError("Token required")

        # Check if this is a competition token
        team_id = self.competition_manager.get_team_id(token) if self.competition_manager else None

        if team_id:
            return self._handle_competition_action(action, token, team_id, params)

        session = self.session_manager.get_session(token)
        if not session:
            raise ValueError("Invalid or expired session")

        self.session_manager.update_activity(token)
        sim = self.simulations.get(token)

        if action == "add_object":
            obj_id = sim.add_object(
                params["type"],
                params["position"],
                size=params.get("size"),
                radius=params.get("radius"),
            )
            return {"object_id": obj_id, "status": "ok"}

        elif action == "move_robot":
            robot_id = params.get("robot_id", 0)
            smooth = params.get("smooth", False)
            if smooth:
                num_waypoints = params.get("num_waypoints", 50)
                sim.move_robot_smooth(robot_id, params["position"], num_waypoints)
                return {"status": "ok"}
            else:
                joint_angles = sim.move_robot(robot_id, params["position"])
                return {"joint_angles": joint_angles, "status": "ok"}

        elif action == "move_joints":
            robot_id = params.get("robot_id", 0)
            sim.move_joints(robot_id, params["angles"])
            return {"status": "ok"}

        elif action == "gripper":
            robot_id = params.get("robot_id", 0)
            sim.gripper(robot_id, params["action"])
            return {"status": "ok"}

        elif action == "get_state":
            robot_id = params.get("robot_id", 0)
            state = sim.get_state(robot_id)
            return {**state, "status": "ok"}

        elif action == "step":
            sim.step(params.get("steps", 1))
            return {"status": "ok"}

        elif action == "reset":
            sim.reset()
            return {"status": "ok"}

        elif action == "reset_robot":
            robot_id = params.get("robot_id", 0)
            sim.reset_to_home(robot_id)
            return {"status": "ok"}

        elif action == "start_recording":
            sim.start_recording()
            return {"status": "ok"}

        elif action == "stop_recording":
            video_data = sim.stop_recording()
            video_base64 = base64.b64encode(video_data).decode("utf-8")
            return {"video_base64": video_base64, "status": "ok"}

        elif action == "get_objects":
            objects = sim.get_objects()
            return {"objects": objects, "status": "ok"}

        elif action == "destroy_env":
            if not token:
                return {"status": "error", "error": "missing token"}

            # Remove from the registry first so the token can't be used
            # by concurrent requests while we're tearing it down.
            sim = self.simulations.pop(token, None)

            if sim is None:
                # Nothing registered under this token. Still attempt to
                # release the session in case it's a dangling allocation.
                self.session_manager.destroy_session(token)
                return {"status": "ok", "note": "no active env for token"}

            # Explicitly release the simulation's resources (GPU memory,
            # physics context, file handles, etc.). Don't let a failure
            # here stop us from freeing the session slot.
            try:
                sim.destroy()
            except Exception as exc:
                # Capture the underlying teardown error instead of masking it.
                print(f"Error destroying simulation for token {token}: {exc!r}")

            # Free the capacity slot last, once the sim is actually gone.
            self.session_manager.destroy_session(token)

            return {"status": "ok"}

        else:
            raise ValueError(f"Unknown action: {action}")

    def _create_env(self, params: Dict[str, Any]) -> Dict[str, Any]:
        scene_name = params.get("scene", "empty")
        token = self.session_manager.create_session(scene_name)

        if not token:
            raise ValueError("Server at capacity, try again later")

        sim = GenesisSimulation(scene_name)
        sim.build()
        self.simulations[token] = sim

        return {"token": token, "status": "ok"}

    def _join_competition(self, params: Dict[str, Any]) -> Dict[str, Any]:
        team_id = params["team_id"]
        token = self.competition_manager.join(team_id)
        return {"token": token, "status": "ok"}

    def _handle_competition_action(
        self, action: str, token: str, team_id: str, params: Dict[str, Any]
    ) -> Dict[str, Any]:
        sim = self.competition_manager.get_simulation()
        if not sim:
            raise ValueError("Competition not active")

        robot_id = self.competition_manager.get_robot_index(team_id)

        # Freeze the inactive robot, unfreeze active robot
        other_robot_id = 1 - robot_id
        sim.freeze_robot(other_robot_id)
        sim.unfreeze_robot(robot_id)

        if action == "leave_competition":
            self.competition_manager.leave(token)
            return {"status": "ok"}

        if action == "end_turn":
            next_turn = self.competition_manager.end_turn(token)
            return {"status": "ok", "next_turn": next_turn}

        if action == "flip_card":
            if not self.competition_manager.can_move(token):
                raise ValueError("Not your turn")

            row = params["row"]
            col = params["col"]

            # Move THIS team's robot to point at the card, then reveal it
            result = sim.flip_card(row, col, robot_id=robot_id)

            # Track flipped cards for match detection
            if not result["already_flipped"]:
                match_result = self.competition_manager.record_flip(
                    token, row, col, result["color_idx"]
                )
                result["match_result"] = match_result

            return {**result, "status": "ok"}

        if action == "get_card_state":
            row = params["row"]
            col = params["col"]
            state = sim.get_card_state(row, col)
            return {**state, "status": "ok"}

        if action == "get_grid_state":
            grid = sim.get_grid_state()
            return {"grid": grid, "status": "ok"}

        if action == "move_robot":
            if not self.competition_manager.can_move(token):
                raise ValueError("Not your turn")
            joint_angles = sim.move_robot(robot_id, params["position"])
            # Auto-step to execute the movement
            sim.step(params.get("steps", 100))
            return {"joint_angles": joint_angles, "status": "ok"}

        if action == "move_joints":
            if not self.competition_manager.can_move(token):
                raise ValueError("Not your turn")
            sim.move_joints(robot_id, params["angles"])
            sim.step(params.get("steps", 100))
            return {"status": "ok"}

        if action == "gripper":
            if not self.competition_manager.can_move(token):
                raise ValueError("Not your turn")
            sim.gripper(robot_id, params["action"])
            sim.step(params.get("steps", 50))
            return {"status": "ok"}

        if action == "get_state":
            state = sim.get_state(robot_id)
            return {**state, "status": "ok"}

        if action == "get_objects":
            objects = sim.get_objects()
            return {"objects": objects, "status": "ok"}

        raise ValueError(f"Unknown competition action: {action}")

    def _handle_admin_action(
        self, action: str, params: Dict[str, Any]
    ) -> Dict[str, Any]:
        password = params.get("password")
        if password != ADMIN_PASSWORD:
            raise ValueError("Invalid admin password")

        if action == "admin_get_status":
            return {
                "sessions": self.session_manager.get_session_count(),
                "competition_active": self.competition_manager.is_active() if self.competition_manager else False,
                "status": "ok",
            }

        if action == "admin_start_competition":
            self.competition_manager.start(
                params["scene"],
                card_layout=params.get("card_layout"),
            )
            return {"status": "ok"}

        if action == "admin_stop_competition":
            self.competition_manager.stop()
            return {"status": "ok"}

        if action == "admin_list_card_images":
            images = []
            if os.path.exists(CARD_IMAGES_DIR):
                images = [
                    f for f in os.listdir(CARD_IMAGES_DIR)
                    if f.lower().endswith(('.png', '.jpg', '.jpeg'))
                ]
            return {"images": images, "status": "ok"}

        if action == "admin_set_card_layout":
            # Store card layout for next competition start
            return {"status": "ok"}

        raise ValueError(f"Unknown admin action: {action}")

    def _send_response(self, data: Dict[str, Any]) -> None:
        response = json.dumps(data).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", len(response))
        self.end_headers()
        self.wfile.write(response)

    def _send_error(self, message: str) -> None:
        response = json.dumps({"status": "error", "message": message}).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", len(response))
        self.end_headers()
        self.wfile.write(response)

    def log_message(self, format, *args):
        pass  # Suppress request logging


class GenesisServer(HTTPServer):
    def __init__(self, port: int = PORT):
        GenesisRequestHandler.session_manager = SessionManager(
            max_sessions=MAX_SESSIONS,
            timeout=SESSION_TIMEOUT,
            cleanup_interval=CLEANUP_INTERVAL,
        )
        GenesisRequestHandler.simulations = {}
        GenesisRequestHandler.competition_manager = CompetitionManager()

        self.allow_reuse_address = True
        super().__init__(("", port), GenesisRequestHandler)

        GenesisRequestHandler.session_manager.start_cleanup_thread()


def get_all_ips():
    """Get all local IP addresses."""
    import socket
    ips = []
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if ip not in ips and not ip.startswith("127."):
                ips.append(ip)
    except Exception:
        pass

    # Also try the route-based detection
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        route_ip = s.getsockname()[0]
        s.close()
        if route_ip not in ips:
            ips.append(route_ip)
    except Exception:
        pass

    return ips if ips else ["127.0.0.1"]


def main():
    import threading
    import itertools
    import time

    all_ips = get_all_ips()

    show_viewer = os.environ.get('GENESIS_SHOW_VIEWER', 'true').lower() == 'true'
    stream_port = int(os.environ.get('GENESIS_STREAM_PORT', '8080'))

    print(f"\n{'='*55}")
    print(f"  Genesis Simulation Server")
    print(f"{'='*55}")
    print(f"  Available IPs:")
    for ip in all_ips:
        print(f"    - Main API: http://{ip}:{PORT}")
        print(f"    - Stream: http://{ip}:{stream_port}")
    print(f"  Backend    : {os.environ.get('GENESIS_BACKEND', 'cpu')}")
    print(f"  Viewer     : {'enabled' if show_viewer else 'disabled'}")
    print(f"{'='*55}")
    print(f"  Example: SimulationClient(\"{all_ips[0]}\", {PORT})")
    print(f"{'='*55}\n")

    server = GenesisServer()

    # Start the streaming server
    from .stream_server import start_stream_server
    start_stream_server(port=stream_port, simulations_dict=GenesisRequestHandler.simulations)

    print("  Server running. Press Ctrl+C to stop.\n")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n\nShutting down...")
        server.shutdown()


if __name__ == "__main__":
    main()
