#!/usr/bin/env python3
"""
PYNQ P2P Test Client

A simple command-line client to test the pynq-p2p messaging system
without needing PYNQ hardware or sensors.

Usage:
    python test_client.py [--server SERVER_IP] [--key KEY]
"""

import argparse
import sys

# Try to import pynqp2p
try:
    import pynqp2p
except ImportError as e:
    print("Error: pynqp2p library not found!")
    print(f"Import error: {e}")
    print("\nTroubleshooting:")
    print("1. Install with: pip3 install ./pynq-p2p")
    print("2. Or install with: pip install -e ./pynq-p2p")
    print("3. Check installation: pip list | grep pynqp2p")
    print("4. Try running from a different directory")
    sys.exit(1)


def print_banner():
    """Print welcome banner."""
    print("=" * 60)
    print("PYNQ P2P Test Client")
    print("=" * 60)
    print()


def print_menu():
    """Print interactive menu."""
    print("\nCommands:")
    print("  1 - Ping server")
    print("  2 - Get my board ID")
    print("  3 - Send message")
    print("  4 - Receive one message")
    print("  5 - Receive all messages")
    print("  6 - Send multiple messages (bulk test)")
    print("  7 - Show current config")
    print("  h - Show this help")
    print("  q - Quit")
    print()


def ping_server():
    """Test server connectivity."""
    print("\n[Ping] Testing server connection...")
    try:
        response = pynqp2p.ping()
        print(f"✓ Server responded: {response}")
        return True
    except Exception as e:
        print(f"✗ Ping failed: {e}")
        return False


def get_board_id():
    """Get and display this board's ID."""
    print("\n[Get ID] Retrieving board identifier...")
    try:
        board_id = pynqp2p.get_id()
        print(f"✓ Your board ID: {board_id}")
        print(f"  Share this ID with others to receive messages")
        return board_id
    except Exception as e:
        print(f"✗ Failed to get board ID: {e}")
        return None


def send_message():
    """Send a message to another board."""
    print("\n[Send Message]")
    recipient = input("Enter recipient board ID: ").strip()
    if not recipient:
        print("✗ Recipient ID required")
        return

    message = input("Enter message: ").strip()
    if not message:
        print("✗ Message required")
        return

    try:
        response = pynqp2p.send(recipient, message)
        print(f"✓ {response}")
    except Exception as e:
        print(f"✗ Send failed: {e}")


def receive_message():
    """Receive one message."""
    print("\n[Receive Message] Checking for messages...")
    try:
        message = pynqp2p.receive()
        if message:
            print(f"✓ Received: {message}")
        else:
            print("  No messages in queue")
    except Exception as e:
        print(f"✗ Receive failed: {e}")


def receive_all_messages():
    """Receive all messages."""
    print("\n[Receive All] Checking for messages...")
    try:
        messages = pynqp2p.receive_all()
        if messages:
            print(f"✓ Received {len(messages)} message(s):")
            for i, msg in enumerate(messages, 1):
                print(f"  {i}. {msg}")
        else:
            print("  No messages in queue")
    except Exception as e:
        print(f"✗ Receive all failed: {e}")


def send_bulk_messages():
    """Send multiple messages for testing."""
    print("\n[Bulk Send]")
    recipient = input("Enter recipient board ID: ").strip()
    if not recipient:
        print("✗ Recipient ID required")
        return

    try:
        count = int(input("How many messages to send? [5]: ").strip() or "5")
    except ValueError:
        print("✗ Invalid number")
        return

    print(f"Sending {count} messages to {recipient}...")
    try:
        for i in range(count):
            message = f"Test message {i+1}/{count}"
            pynqp2p.send(recipient, message)
            print(f"  Sent {i+1}/{count}")
        print(f"✓ Successfully sent {count} messages")
    except Exception as e:
        print(f"✗ Bulk send failed at message {i+1}: {e}")


def show_config():
    """Show current configuration."""
    print("\n[Configuration]")
    try:
        ip = pynqp2p.get_ip()
        key = pynqp2p.get_key()
        board_id = pynqp2p.get_id()

        print(f"  Server IP:  {ip}")
        print(f"  Auth Key:   {'*' * len(key)}")
        print(f"  Board ID:   {board_id}")
    except Exception as e:
        print(f"✗ Failed to read config: {e}")
        print("  Have you registered yet? (pynqp2p.register())")


def interactive_mode():
    """Run interactive command loop."""
    print_banner()
    print("Type 'h' for help, 'q' to quit")

    while True:
        try:
            command = input("\n> ").strip().lower()

            if command == 'q' or command == 'quit':
                print("Goodbye!")
                break
            elif command == 'h' or command == 'help':
                print_menu()
            elif command == '1':
                ping_server()
            elif command == '2':
                get_board_id()
            elif command == '3':
                send_message()
            elif command == '4':
                receive_message()
            elif command == '5':
                receive_all_messages()
            elif command == '6':
                send_bulk_messages()
            elif command == '7':
                show_config()
            elif command == '':
                continue
            else:
                print(f"Unknown command: '{command}'. Type 'h' for help.")

        except KeyboardInterrupt:
            print("\n\nInterrupted. Type 'q' to quit.")
        except EOFError:
            print("\n\nGoodbye!")
            break


def run_quick_test(server_ip, key, recipient_id=None):
    """Run automated quick test."""
    print_banner()
    print("Running quick test sequence...\n")

    # Register
    print(f"[1/5] Registering with server {server_ip}...")
    try:
        pynqp2p.register(server_ip, key)
        print("✓ Registered")
    except Exception as e:
        print(f"✗ Registration failed: {e}")
        return False

    # Ping
    print("\n[2/5] Pinging server...")
    if not ping_server():
        return False

    # Get ID
    print("\n[3/5] Getting board ID...")
    board_id = get_board_id()
    if not board_id:
        return False

    # Send to self
    print("\n[4/5] Sending test message to self...")
    try:
        pynqp2p.send(board_id, "Test message from test_client.py")
        print("✓ Message sent to self")
    except Exception as e:
        print(f"✗ Send failed: {e}")
        return False

    # Receive
    print("\n[5/5] Receiving message...")
    receive_message()

    print("\n" + "=" * 60)
    print("Quick test complete!")
    print("=" * 60)

    if recipient_id:
        print(f"\nTo send to another board, use recipient ID: {recipient_id}")

    return True


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='PYNQ P2P Test Client',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Interactive mode (register first)
  python test_client.py

  # Quick test with auto-registration
  python test_client.py --server 192.168.1.100 --key bootcamp2024 --test

  # Register and enter interactive mode
  python test_client.py --server 192.168.1.100 --key bootcamp2024
        """
    )

    parser.add_argument(
        '--server',
        help='Server IP address (e.g., 192.168.1.100 or localhost:5000)'
    )

    parser.add_argument(
        '--key',
        default='bootcamp2024',
        help='Authentication key (default: bootcamp2024)'
    )

    parser.add_argument(
        '--test',
        action='store_true',
        help='Run quick automated test instead of interactive mode'
    )

    parser.add_argument(
        '--recipient',
        help='Recipient board ID for quick test mode'
    )

    args = parser.parse_args()

    # If server specified, register
    if args.server:
        # Add http:// if not present
        server = args.server
        if not server.startswith('http://') and not server.startswith('https://'):
            server = f'http://{server}'

        print(f"Registering with server: {server}")
        print(f"Using key: {'*' * len(args.key)}")

        try:
            pynqp2p.register(server, args.key)
            print("✓ Registration successful\n")
        except Exception as e:
            print(f"✗ Registration failed: {e}")
            sys.exit(1)

    # Run in appropriate mode
    if args.test:
        if not args.server:
            print("Error: --test mode requires --server")
            sys.exit(1)
        success = run_quick_test(server, args.key, args.recipient)
        sys.exit(0 if success else 1)
    else:
        interactive_mode()


if __name__ == '__main__':
    main()
