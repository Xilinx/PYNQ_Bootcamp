import os

PORT = int(os.environ.get("GENESIS_PORT", 9002))
STREAM_PORT = int(os.environ.get("GENESIS_STREAM_PORT", 9005))  # Video streaming server port
BACKEND = os.environ.get("GENESIS_BACKEND", "amdgpu")  # cpu, gpu, cuda, amdgpu, metal
SHOW_VIEWER = os.environ.get("GENESIS_SHOW_VIEWER", "true").lower() == "true"
MAX_SESSIONS = int(os.environ.get("GENESIS_MAX_SESSIONS", 10))
SESSION_TIMEOUT = int(os.environ.get("GENESIS_SESSION_TIMEOUT", 7200))  # 2 hours
CLEANUP_INTERVAL = int(os.environ.get("GENESIS_CLEANUP_INTERVAL", 300))  # 5 minutes
SCENE_RELOAD_INTERVAL = int(os.environ.get("GENESIS_SCENE_RELOAD_INTERVAL", 30))
ADMIN_PASSWORD = os.environ.get("GENESIS_ADMIN_PASSWORD", "admin123")
COMPETITION_STEP_RATE = int(os.environ.get("GENESIS_COMPETITION_STEP_RATE", 60))

SCENES_DIR = os.path.join(os.path.dirname(__file__), "..", "scenes")
ASSETS_DIR = os.path.join(os.path.dirname(__file__), "..", "assets")
CARD_IMAGES_DIR = os.path.join(ASSETS_DIR, "card_images")
