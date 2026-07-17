# PYNQ P2P - Board-to-Board Communication

A simple message-passing system for PYNQ boards to communicate over HTTP.

## Architecture

- **Client Library** (`pynqp2p/`) - Installed on PYNQ boards
- **Server** (`server.py`) - Central message broker run by instructor

## Quick Start

### Testing Without Hardware

Use the test client to verify the server and messaging system:

```bash
# Terminal 1 - Start the server
python server.py --host localhost --port 5000 --key testkey

# Terminal 2 - Run quick test
python test_client.py --server localhost:5000 --key testkey --test

# Or interactive mode
python test_client.py --server localhost:5000 --key testkey
```

The test client provides an interactive menu to:
- Ping the server
- Get your board ID
- Send/receive messages
- Test bulk messaging

### Server Setup (Instructor)

1. Install dependencies:
```bash
pip3 install -r server-requirements.txt
```

2. Run the server:
```bash
# Basic usage (localhost only)
python server.py

# Accessible from network
python server.py --host 0.0.0.0 --port 5000 --key bootcamp2024

# Custom key for security
python server.py --host 0.0.0.0 --key mySecretKey123
```

3. Share with students:
   - Server IP address (e.g., `192.168.1.100`)
   - Authentication key (e.g., `bootcamp2024`)

### Client Setup (Students on PYNQ Boards)

1. Install the client library:
```python
!pip3 install ./pynq-p2p
```

2. Register with the server:
```python
import pynqp2p

# Use the IP and key provided by instructor
pynqp2p.register('192.168.1.100', 'bootcamp2024')
```

3. Get your board ID:
```python
my_id = pynqp2p.get_id()
print(f"My board ID: {my_id}")
# Share this ID with your partner
```

4. Send and receive messages:
```python
# Send a message
partner_id = "b8:27:eb:12:34:56"  # Partner's board ID
pynqp2p.send(partner_id, "Hello from my board!")

# Receive messages
message = pynqp2p.receive()  # Get one message
print(message)

# Or receive all queued messages
messages = pynqp2p.receive_all()  # Get all messages
for msg in messages:
    print(msg)
```

## API Reference

### Client API

**Configuration:**
- `register(ip, key)` - Connect to server with IP and authentication key
- `get_id()` - Get this board's unique MAC address

**Communication:**
- `ping()` - Test connection to server
- `send(recipient_id, message)` - Send message to a board
- `receive()` - Get oldest message (deletes it from queue)
- `receive_all()` - Get all messages (deletes them from queue)

### Server Endpoints

**Message Passing:**
- `POST /ping` - Health check
- `POST /send` - Send message to board
- `GET /receive` - Receive one message
- `GET /receive_all` - Receive all messages

**Administration:**
- `GET /stats` - View server statistics
- `POST /clear` - Clear message queues
- `GET /` - Server info page

## Server Administration

### View Statistics
```bash
curl http://localhost:5000/stats
```

### Clear All Queues
```bash
curl -X POST -d "key=bootcamp2024" http://localhost:5000/clear
```

### Clear Specific Board Queue
```bash
curl -X POST -d "key=bootcamp2024&id=b8:27:eb:12:34:56" http://localhost:5000/clear
```

## Example Use Cases

### Temperature Sensor Network
```python
# Board A - Temperature sensor
import pynqp2p
from pynq.lib.arduino import Grove_TH02, ARDUINO_GROVE_I2C

temp_c, humidity = th02.read()
temp_f = temp_c * (9/5) + 32

# Send to monitoring board
pynqp2p.send(monitor_board_id, f"temp:{temp_f},humidity:{humidity}")
```

```python
# Board B - Monitor/display
import pynqp2p

messages = pynqp2p.receive_all()
for msg in messages:
    if msg.startswith("temp:"):
        data = msg.split(':')[1]
        temp, humidity = data.split(',')
        print(f"Received: {temp}°F, {humidity}% humidity")
```

### Collaborative Computation
```python
# Board A - Send Celsius measurement
temp_c, _ = th02.read()
pynqp2p.send(partner_id, str(temp_c))

# Board B - Convert to Fahrenheit
temp_c = float(pynqp2p.receive())
temp_f = temp_c * (9/5) + 32
pynqp2p.send(original_sender_id, f"Your temperature in F: {temp_f}")
```

## Security Notes

⚠️ **This is designed for educational use in a sandboxed network.**

- Uses HTTP (not HTTPS) - no encryption
- Shared secret key authentication - all students use same key
- No message encryption or signing
- Suitable for classroom/lab environment only
- **Do not use for production or sensitive data**

## Troubleshooting

**"Connection refused"**
- Verify server is running
- Check IP address is correct
- Ensure firewall allows traffic on the port

**"Unauthorized"**
- Verify authentication key matches server
- Check `~/pynqp2p-key` file contains correct key

**No messages received**
- Verify recipient ID is correct (use `get_id()`)
- Check sender used the right ID
- Messages are deleted after retrieval - can only receive once

**Server not accessible from network**
- Use `--host 0.0.0.0` to bind to all interfaces
- Check firewall settings on server machine
- Verify boards are on same network as server

## Architecture Notes

**Message Queue Behavior:**
- FIFO (First In, First Out) delivery
- Destructive reads (messages deleted when retrieved)
- In-memory storage (messages lost on server restart)
- No message persistence or replay
- No delivery guarantees

**Threading:**
- Server uses Flask with thread-safe message queue access
- Lock protects queue modifications
- Suitable for classroom scale (10-20 boards)

## License

Created for Xilinx PYNQ Bootcamp educational purposes.
