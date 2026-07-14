import uuid
import time
import threading
from typing import Dict, Any, Optional, List

from .simulation import GenesisSimulation
from .config import COMPETITION_STEP_RATE


class CompetitionManager:
    def __init__(self):
        self._active = False
        self._simulation: Optional[GenesisSimulation] = None
        self._teams: Dict[str, str] = {}  # team_id -> token
        self._tokens: Dict[str, str] = {}  # token -> team_id
        self._current_turn: Optional[str] = None
        self._current_turn_idx: int = 0
        self._scores: Dict[str, int] = {}
        self._turn_based = False
        self._step_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._scene_name: Optional[str] = None
        self._card_layout: Optional[Dict[str, Any]] = None
        # Card flip tracking
        self._flipped_this_turn: List[tuple] = []  # [(row, col, color_idx), ...]

    def start(
        self,
        scene_name: str,
        card_layout: Optional[Dict[str, Any]] = None,
    ) -> None:
        with self._lock:
            if self._active:
                raise ValueError("Competition already active")

            self._scene_name = scene_name
            self._card_layout = card_layout

            self._simulation = GenesisSimulation(scene_name, card_layout=card_layout)
            self._simulation.build()

            # Determine turn-based mode based on scene
            self._turn_based = "card_flip" in scene_name

            # Initialize default teams for competition scenes
            if "competition" in scene_name:
                self._scores["team_red"] = 0
                self._scores["team_blue"] = 0
                self._current_turn = "team_red"

            self._active = True

            # Start auto-stepping if not turn-based
            if not self._turn_based:
                self._start_auto_step()

    def restart(self, scene_name: Optional[str] = None,
                card_layout: Optional[Dict[str, Any]] = None) -> None:
        """Stop the current competition (if any) and start a fresh one.

        Defaults to the same scene/layout as the current game unless overridden.
        """
        prev_scene = self._scene_name
        prev_layout = self._card_layout
        if self._active:
            self.stop()
        self.start(scene_name or prev_scene, card_layout=card_layout or prev_layout)

    def stop(self) -> None:
            with self._lock:
                self._active = False
                self._stop_auto_step()
                if self._simulation is not None:
                    try:
                        self._simulation.destroy()
                    except Exception as exc:
                        print(f"Warning: sim teardown failed during stop: {exc!r}")
                self._simulation = None
                self._teams.clear()
                self._tokens.clear()
                self._scores.clear()
                self._current_turn = None
                self._current_turn_idx = 0
                self._scene_name = None
                self._card_layout = None
                self._flipped_this_turn.clear()

    def join(self, team_id: str) -> Optional[str]:
        with self._lock:
            if not self._active:
                raise ValueError("No competition active")

            if team_id in self._teams:
                raise ValueError(f"Team {team_id} already joined")

            token = str(uuid.uuid4())
            self._teams[team_id] = token
            self._tokens[token] = team_id

            if team_id not in self._scores:
                self._scores[team_id] = 0

            return token

    def leave(self, token: str) -> None:
        print(f"leave_compo", flush=True)
        with self._lock:
            team_id = self._tokens.pop(token, None)
            if team_id:
                self._teams.pop(team_id, None)

    def get_team_id(self, token: str) -> Optional[str]:
        with self._lock:
            return self._tokens.get(token)

    def get_simulation(self) -> Optional[GenesisSimulation]:
        return self._simulation

    def get_robot_index(self, team_id: str) -> int:
        """Get the robot index for a team."""
        if team_id == "team_red":
            return 0
        elif team_id == "team_blue":
            return 1
        return 0

    def get_state(self) -> Dict[str, Any]:
        print(f"get_state()", flush=True)
        #print(f"\r  [{client_ip}] action={action:<25}", flush=True)
        with self._lock:
            if not self._active or not self._simulation:
                return {"status": "error", "message": "No competition active"}

            robots_state = {}
            for i, robot in enumerate(self._simulation.robots):
                state = self._simulation.get_state(i)
                team_id = "team_red" if i == 0 else "team_blue"
                robots_state[team_id] = state

            objects = self._simulation.get_objects()

            return {
                "robots": robots_state,
                "objects": objects,
                "current_turn": self._current_turn,
                "scores": self._scores.copy(),
                "turn_based": self._turn_based,
                "status": "ok",
            }

    def record_flip(self, token: str, row: int, col: int, color_idx: int) -> "Dict[str, Any]":
        """Record a card flip and resolve the pair when two are flipped.

        Memory-game rules:
        * match    -> score++, cards stay revealed, SAME team goes again.
        * mismatch -> ~2s memory pause, re-cover both cards, turn passes.

        Returns a dict with cards_flipped, matched, turn_ended, and
        same_team_again so the client can tell what happened.
        """
        # ~2 second memory pause (dt=0.01 -> 200 steps). Tune here.
        MEMORY_PAUSE_STEPS = 200

        with self._lock:
            self._flipped_this_turn.append((row, col, color_idx))

            result = {
                "cards_flipped": len(self._flipped_this_turn),
                "matched": False,
                "turn_ended": False,
                "same_team_again": False,
            }

            if len(self._flipped_this_turn) == 2:
                _, _, color1 = self._flipped_this_turn[0]
                _, _, color2 = self._flipped_this_turn[1]

                team_id = self._tokens.get(token)
                sim = self._simulation

                if color1 == color2:
                    # --- MATCH: score, keep cards revealed, same team again ---
                    result["matched"] = True
                    if team_id and team_id in self._scores:
                        self._scores[team_id] += 1
                    if sim and sim.card_grid:
                        for r, c, _ in self._flipped_this_turn:
                            sim.card_grid[r][c]["matched"] = True

                    # Reset the pair buffer but DO NOT advance the turn.
                    self._flipped_this_turn = []
                    result["turn_ended"] = False
                    result["same_team_again"] = True

                else:
                    # --- MISMATCH: pause so opponent can memorize, then re-cover ---
                    if sim:
                        sim.step(MEMORY_PAUSE_STEPS)   # both colors stay visible
                        for r, c, _ in self._flipped_this_turn:
                            sim.unflip_card(r, c)      # teleport covers back

                    self._flipped_this_turn = []
                    result["turn_ended"] = True

                    # Pass the turn to the other team.
                    team_ids = ["team_red", "team_blue"]
                    self._current_turn_idx = (self._current_turn_idx + 1) % 2
                    self._current_turn = team_ids[self._current_turn_idx]

            return result

    def end_turn(self, token: str) -> str:
        with self._lock:
            if not self._turn_based:
                raise ValueError("Not in turn-based mode")

            team_id = self._tokens.get(token)
            if not team_id:
                raise ValueError("Invalid token")

            if team_id != self._current_turn:
                raise ValueError("Not your turn")

            # Flip back any cards that weren't matched this turn
            if self._flipped_this_turn:
                sim = self._simulation
                if sim:
                    for r, c, _ in self._flipped_this_turn:
                        sim.unflip_card(r, c)
                self._flipped_this_turn = []

            # Rotate turn
            team_ids = ["team_red", "team_blue"]
            self._current_turn_idx = (self._current_turn_idx + 1) % 2
            self._current_turn = team_ids[self._current_turn_idx]

            return self._current_turn

    def is_active(self) -> bool:
        with self._lock:
            return self._active

    def is_turn_based(self) -> bool:
        with self._lock:
            return self._turn_based

    def can_move(self, token: str) -> bool:
        with self._lock:
            if not self._turn_based:
                return True

            team_id = self._tokens.get(token)
            return team_id == self._current_turn

    def update_score(self, team_id: str, points: int) -> None:
        with self._lock:
            if team_id in self._scores:
                self._scores[team_id] += points

    def _start_auto_step(self) -> None:
        self._step_thread = threading.Thread(target=self._auto_step_loop, daemon=True)
        self._step_thread.start()

    def _stop_auto_step(self) -> None:
        if self._step_thread:
            self._step_thread = None

    def _auto_step_loop(self) -> None:
        interval = 1.0 / COMPETITION_STEP_RATE
        while self._active and self._step_thread:
            if self._simulation:
                self._simulation.step(1)
            time.sleep(interval)
