#!/usr/bin/env python3
"""
PYNQ P2P Server
A simple HTTP server for board-to-board message passing in PYNQ bootcamp.

Usage:
    python server.py [--host HOST] [--port PORT] [--key KEY]

Example:
    python server.py --host 0.0.0.0 --port 5000 --key bootcamp2024
"""

from flask import Flask, request, jsonify
from datetime import datetime
import argparse
import logging
from collections import defaultdict
from threading import Lock

app = Flask(__name__)

# Configuration
SECRET_KEY = "bootcamp2024"  # Default key, should be changed via CLI

# Message storage: {board_id: [messages]}
message_queues = defaultdict(list)
message_lock = Lock()

# Statistics
stats = {
    'total_messages': 0,
    'total_pings': 0,
    'active_boards': set()
}

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def verify_key(key):
    """Verify the authentication key."""
    return key == SECRET_KEY


def get_timestamp():
    """Get current timestamp for logging."""
    return datetime.now().strftime('%Y-%m-%d %H:%M:%S')


@app.route('/ping', methods=['POST'])
def ping():
    """
    Health check endpoint.

    Expected data:
        - key: Authentication key
        - id: Board MAC address

    Returns:
        - "pong" if authenticated
        - 401 if authentication fails
    """
    key = request.form.get('key', '')
    board_id = request.form.get('id', '')

    if not verify_key(key):
        logger.warning(f"Unauthorized ping attempt from {request.remote_addr}")
        return "Unauthorized", 401

    stats['total_pings'] += 1
    stats['active_boards'].add(board_id)

    logger.info(f"Ping from board {board_id} ({request.remote_addr})")
    return "pong"


@app.route('/send', methods=['POST'])
def send():
    """
    Send a message to a specific board.

    Expected data:
        - key: Authentication key
        - id: Recipient board MAC address
        - message: Message text

    Returns:
        - Success confirmation or error message
    """
    key = request.form.get('key', '')
    recipient_id = request.form.get('id', '')
    message = request.form.get('message', '')

    if not verify_key(key):
        logger.warning(f"Unauthorized send attempt from {request.remote_addr}")
        return "Unauthorized", 401

    if not recipient_id:
        return "Recipient ID required", 400

    if not message:
        return "Message required", 400

    # Add message to recipient's queue
    with message_lock:
        message_queues[recipient_id].append(message)
        stats['total_messages'] += 1
        queue_size = len(message_queues[recipient_id])

    logger.info(f"Message queued for {recipient_id} (queue size: {queue_size})")
    logger.debug(f"Message content: {message[:50]}...")

    return f"Message sent to {recipient_id}"


@app.route('/receive', methods=['GET'])
def receive():
    """
    Receive the oldest message for a board (FIFO).
    Message is removed from the queue after retrieval (destructive read).

    Expected data:
        - key: Authentication key
        - id: Board MAC address

    Returns:
        - Message text or empty string if no messages
    """
    key = request.form.get('key', '')
    board_id = request.form.get('id', '')

    if not verify_key(key):
        logger.warning(f"Unauthorized receive attempt from {request.remote_addr}")
        return "Unauthorized", 401

    if not board_id:
        return "Board ID required", 400

    with message_lock:
        if board_id in message_queues and message_queues[board_id]:
            message = message_queues[board_id].pop(0)
            remaining = len(message_queues[board_id])
            logger.info(f"Message retrieved by {board_id} ({remaining} remaining)")
            return message
        else:
            logger.debug(f"No messages for {board_id}")
            return ""


@app.route('/receive_all', methods=['GET'])
def receive_all():
    """
    Receive all messages for a board.
    Messages are removed from the queue after retrieval (destructive read).

    Expected data:
        - key: Authentication key
        - id: Board MAC address

    Returns:
        - Newline-delimited messages (with trailing newline)
        - Empty string if no messages
    """
    key = request.form.get('key', '')
    board_id = request.form.get('id', '')

    if not verify_key(key):
        logger.warning(f"Unauthorized receive_all attempt from {request.remote_addr}")
        return "Unauthorized", 401

    if not board_id:
        return "Board ID required", 400

    with message_lock:
        if board_id in message_queues and message_queues[board_id]:
            messages = message_queues[board_id][:]
            message_queues[board_id] = []
            count = len(messages)
            logger.info(f"All {count} message(s) retrieved by {board_id}")
            # Return newline-delimited with trailing newline (matches client expectation)
            return '\n'.join(messages) + '\n'
        else:
            logger.debug(f"No messages for {board_id}")
            return ""


@app.route('/stats', methods=['GET'])
def get_stats():
    """
    Get server statistics (admin endpoint).

    Returns:
        - JSON with server statistics
    """
    with message_lock:
        queued_messages = sum(len(queue) for queue in message_queues.values())
        board_queues = {board_id: len(queue)
                       for board_id, queue in message_queues.items()
                       if queue}

    return jsonify({
        'total_messages_sent': stats['total_messages'],
        'total_pings': stats['total_pings'],
        'active_boards': len(stats['active_boards']),
        'queued_messages': queued_messages,
        'board_queues': board_queues,
        'timestamp': get_timestamp()
    })


@app.route('/clear', methods=['POST'])
def clear_queue():
    """
    Clear message queue for a specific board (admin endpoint).

    Expected data:
        - key: Authentication key
        - id: Board MAC address (optional - if omitted, clears all)

    Returns:
        - Confirmation message
    """
    key = request.form.get('key', '')
    board_id = request.form.get('id', '')

    if not verify_key(key):
        logger.warning(f"Unauthorized clear attempt from {request.remote_addr}")
        return "Unauthorized", 401

    with message_lock:
        if board_id:
            cleared = len(message_queues[board_id])
            message_queues[board_id] = []
            logger.info(f"Cleared {cleared} message(s) for {board_id}")
            return f"Cleared {cleared} message(s) for {board_id}"
        else:
            total_cleared = sum(len(queue) for queue in message_queues.values())
            message_queues.clear()
            logger.info(f"Cleared all queues ({total_cleared} total messages)")
            return f"Cleared all queues ({total_cleared} total messages)"


@app.route('/', methods=['GET'])
def index():
    """
    Server information page.
    """
    return f"""
    <html>
    <head><title>PYNQ P2P Server</title></head>
    <body>
        <h1>PYNQ P2P Message Server</h1>
        <p>Server is running at {get_timestamp()}</p>
        <h2>Endpoints:</h2>
        <ul>
            <li><code>POST /ping</code> - Health check</li>
            <li><code>POST /send</code> - Send message</li>
            <li><code>GET /receive</code> - Receive one message</li>
            <li><code>GET /receive_all</code> - Receive all messages</li>
            <li><code>GET /stats</code> - Server statistics</li>
            <li><code>POST /clear</code> - Clear message queues</li>
        </ul>
        <h2>Current Stats:</h2>
        <ul>
            <li>Total messages sent: {stats['total_messages']}</li>
            <li>Total pings: {stats['total_pings']}</li>
            <li>Active boards: {len(stats['active_boards'])}</li>
        </ul>
        <p><a href="/stats">View detailed stats (JSON)</a></p>
    </body>
    </html>
    """


def main():
    """Main entry point for the server."""
    parser = argparse.ArgumentParser(
        description='PYNQ P2P Message Server',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run with defaults (localhost:5000)
  python server.py

  # Run on all interfaces (accessible from network)
  python server.py --host 0.0.0.0 --port 5000

  # Use custom authentication key
  python server.py --key mySecretKey123

  # Production settings
  python server.py --host 0.0.0.0 --port 80 --key bootcamp2024 --debug False
        """
    )

    parser.add_argument(
        '--host',
        default='127.0.0.1',
        help='Host to bind to (default: 127.0.0.1, use 0.0.0.0 for all interfaces)'
    )

    parser.add_argument(
        '--port',
        type=int,
        default=5000,
        help='Port to bind to (default: 5000)'
    )

    parser.add_argument(
        '--key',
        default='bootcamp2024',
        help='Authentication key (default: bootcamp2024)'
    )

    parser.add_argument(
        '--debug',
        type=lambda x: x.lower() in ('true', '1', 'yes'),
        default=False,
        help='Enable debug mode (default: False)'
    )

    args = parser.parse_args()

    # Set global secret key
    global SECRET_KEY
    SECRET_KEY = args.key

    logger.info("=" * 60)
    logger.info("PYNQ P2P Message Server Starting")
    logger.info("=" * 60)
    logger.info(f"Host: {args.host}")
    logger.info(f"Port: {args.port}")
    logger.info(f"Authentication Key: {'*' * len(args.key)}")
    logger.info(f"Debug Mode: {args.debug}")
    logger.info("=" * 60)
    logger.info("\nClients should use:")
    logger.info(f"  pynqp2p.register('http://{args.host}:{args.port}', '{args.key}')")
    logger.info("=" * 60)

    # Run the server
    app.run(host=args.host, port=args.port, debug=args.debug)


if __name__ == '__main__':
    main()
