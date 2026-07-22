#!/usr/bin/env python3
"""
Entry point for Genesis simulation server.

Usage:
    python run_server.py

Environment variables:
    GENESIS_PORT: Server port (default: 9002)
    GENESIS_ADMIN_PASSWORD: Admin password (default: admin123)
    GENESIS_MAX_SESSIONS: Maximum concurrent sessions (default: 10)
    GENESIS_SESSION_TIMEOUT: Session timeout in seconds (default: 7200)
"""

import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from genesis_server.server import main

if __name__ == "__main__":
    main()
