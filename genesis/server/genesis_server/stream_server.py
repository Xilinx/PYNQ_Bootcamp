"""
Web-based live viewer for Genesis simulations.

This module provides a Flask-based streaming server that runs alongside
the main Genesis server, allowing students to view their robot simulations
in real-time via a web browser.
"""

import base64
import time
import threading
from typing import Dict, Optional
from flask import Flask, Response, render_template_string, jsonify
import numpy as np

# Will be set by the main server
simulations: Dict[str, 'GenesisSimulation'] = {}

app = Flask(__name__)
app.config['THREADED'] = True


VIEWER_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Robot Simulation - Live View</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }

        body {
            background: linear-gradient(135deg, #1e3c72 0%, #2a5298 100%);
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Arial, sans-serif;
            color: white;
            min-height: 100vh;
            display: flex;
            flex-direction: column;
        }

        .header {
            background: rgba(0, 0, 0, 0.3);
            padding: 20px;
            text-align: center;
            border-bottom: 3px solid #4CAF50;
        }

        .header h1 {
            font-size: 32px;
            margin-bottom: 5px;
            text-shadow: 2px 2px 4px rgba(0, 0, 0, 0.5);
        }

        .header .subtitle {
            font-size: 14px;
            opacity: 0.8;
        }

        .container {
            flex: 1;
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            padding: 20px;
        }

        .video-container {
            background: rgba(0, 0, 0, 0.5);
            border: 5px solid #4CAF50;
            border-radius: 15px;
            overflow: hidden;
            box-shadow: 0 10px 40px rgba(0, 0, 0, 0.5);
            max-width: 95%;
            max-height: 70vh;
        }

        .video-container img {
            display: block;
            width: 100%;
            height: auto;
        }

        .info-panel {
            background: rgba(0, 0, 0, 0.3);
            border-radius: 10px;
            padding: 15px 25px;
            margin-top: 20px;
            display: flex;
            gap: 30px;
            flex-wrap: wrap;
            justify-content: center;
        }

        .info-item {
            display: flex;
            flex-direction: column;
            align-items: center;
        }

        .info-item .label {
            font-size: 12px;
            opacity: 0.7;
            margin-bottom: 5px;
        }

        .info-item .value {
            font-size: 18px;
            font-weight: bold;
            color: #4CAF50;
        }

        .status-indicator {
            width: 12px;
            height: 12px;
            border-radius: 50%;
            background: #4CAF50;
            display: inline-block;
            margin-right: 5px;
            animation: pulse 2s infinite;
        }

        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.5; }
        }

        .instructions {
            background: rgba(255, 255, 255, 0.1);
            border-left: 4px solid #4CAF50;
            padding: 15px;
            margin-top: 20px;
            border-radius: 5px;
            max-width: 600px;
        }

        .instructions h3 {
            margin-bottom: 10px;
            font-size: 16px;
        }

        .instructions p {
            font-size: 14px;
            line-height: 1.6;
            opacity: 0.9;
        }

        .error-container {
            background: rgba(244, 67, 54, 0.2);
            border: 2px solid #f44336;
            border-radius: 10px;
            padding: 30px;
            max-width: 600px;
            text-align: center;
        }

        .error-container h2 {
            color: #f44336;
            margin-bottom: 15px;
        }

        @media (max-width: 768px) {
            .header h1 {
                font-size: 24px;
            }

            .info-panel {
                gap: 15px;
            }
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>🤖 Robot Simulation - Live View</h1>
        <p class="subtitle">Session: {{ token[:8] }}...</p>
    </div>

    <div class="container">
        {% if error %}
        <div class="error-container">
            <h2>⚠️ {{ error_title }}</h2>
            <p>{{ error_message }}</p>
        </div>
        {% else %}
        <div class="video-container">
            <img src="{{ url_for('stream_video', token=token) }}"
                 alt="Robot simulation live stream"
                 id="video-stream">
        </div>

        <div class="info-panel">
            <div class="info-item">
                <span class="label">Status</span>
                <span class="value">
                    <span class="status-indicator"></span>
                    Live
                </span>
            </div>
            <div class="info-item">
                <span class="label">Scene</span>
                <span class="value">{{ scene_name }}</span>
            </div>
            <div class="info-item">
                <span class="label">Robots</span>
                <span class="value">{{ num_robots }}</span>
            </div>
        </div>

        <div class="instructions">
            <h3>💡 Tips</h3>
            <p>
                Keep this tab open while you code in Jupyter!
                You'll see your robot move in real-time as you run commands.
                The stream updates automatically at ~30 FPS.
            </p>
        </div>
        {% endif %}
    </div>

    <script>
        // Detect if stream fails to load
        const img = document.getElementById('video-stream');
        if (img) {
            img.onerror = function() {
                document.querySelector('.video-container').innerHTML =
                    '<div style="padding: 40px; text-align: center;">' +
                    '<h3 style="color: #f44336;">Stream Unavailable</h3>' +
                    '<p style="opacity: 0.8;">The simulation may have ended or the server restarted.</p>' +
                    '<p style="margin-top: 10px;"><a href="" style="color: #4CAF50;">Refresh Page</a></p>' +
                    '</div>';
            };
        }
    </script>
</body>
</html>
"""


@app.route('/')
def index():
    """Landing page showing available streams."""
    active_sessions = []
    for token, sim in simulations.items():
        if sim._initialized:
            active_sessions.append({
                'token': token,
                'token_short': token[:8],
                'scene': sim.scene_name,
                'num_robots': len(sim.robots)
            })

    html = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Genesis Live Viewer</title>
        <style>
            body {
                background: linear-gradient(135deg, #1e3c72 0%, #2a5298 100%);
                font-family: Arial, sans-serif;
                color: white;
                padding: 40px;
            }
            h1 { text-align: center; margin-bottom: 30px; }
            .sessions {
                max-width: 800px;
                margin: 0 auto;
            }
            .session-card {
                background: rgba(0, 0, 0, 0.3);
                padding: 20px;
                margin: 15px 0;
                border-radius: 10px;
                border-left: 5px solid #4CAF50;
            }
            .session-card a {
                color: #4CAF50;
                text-decoration: none;
                font-size: 18px;
            }
            .empty {
                text-align: center;
                opacity: 0.7;
            }
        </style>
    </head>
    <body>
        <h1>🤖 Genesis Live Viewer</h1>
        <div class="sessions">
    """

    if active_sessions:
        for session in active_sessions:
            html += f"""
            <div class="session-card">
                <a href="/view/{session['token']}">
                    Session {session['token_short']} - {session['scene']}
                </a>
                <div style="font-size: 14px; opacity: 0.8; margin-top: 5px;">
                    Robots: {session['num_robots']}
                </div>
            </div>
            """
    else:
        html += '<p class="empty">No active sessions. Create an environment to start streaming!</p>'

    html += """
        </div>
    </body>
    </html>
    """

    return html


@app.route('/view/<token>')
def view_simulation(token):
    """Viewer page for a specific simulation."""
    sim = simulations.get(token)

    if not sim:
        return render_template_string(
            VIEWER_TEMPLATE,
            token=token,
            error=True,
            error_title="Session Not Found",
            error_message=f"No active simulation found for this session. The session may have expired or been destroyed."
        )

    if not sim._initialized:
        return render_template_string(
            VIEWER_TEMPLATE,
            token=token,
            error=True,
            error_title="Simulation Initializing",
            error_message="The simulation is still starting up. Please refresh in a moment."
        )

    return render_template_string(
        VIEWER_TEMPLATE,
        token=token,
        scene_name=sim.scene_name,
        num_robots=len(sim.robots),
        error=False
    )


@app.route('/stream/<token>')
def stream_video(token):
    """MJPEG stream endpoint for a specific simulation."""
    sim = simulations.get(token)

    if not sim or not sim._initialized:
        # Return a placeholder error image
        def generate_error():
            import io
            from PIL import Image, ImageDraw, ImageFont

            img = Image.new('RGB', (640, 480), color=(30, 30, 30))
            draw = ImageDraw.Draw(img)
            draw.text((200, 220), "Session Not Found", fill=(255, 100, 100))

            buffer = io.BytesIO()
            img.save(buffer, format='JPEG')

            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.getvalue() + b'\r\n')

        return Response(generate_error(),
                        mimetype='multipart/x-mixed-replace; boundary=frame')

    def generate_frames():
        """Generate frames from the simulation camera."""
        frame_delay = 1.0 / 30.0  # Target 30 FPS

        # Start recording mode to enable frame capture
        sim = simulations.get(token)
        if not sim or not sim._initialized or not sim.camera:
            return

        # Put camera in recording mode (but we won't save)
        sim.camera.start_recording()

        try:
            while True:
                try:
                    # Check if simulation still exists
                    if token not in simulations or not simulations[token]._initialized:
                        break

                    sim = simulations[token]

                    if sim.camera:
                        # Render current frame
                        sim.camera.render()

                        # Access the recorded images buffer
                        if hasattr(sim.camera, '_recorded_imgs') and len(sim.camera._recorded_imgs) > 0:
                            # Get the most recent frame
                            rgba = sim.camera._recorded_imgs[-1]

                            # Clear old frames to prevent memory buildup
                            if len(sim.camera._recorded_imgs) > 5:
                                sim.camera._recorded_imgs = sim.camera._recorded_imgs[-2:]
                        else:
                            time.sleep(frame_delay)
                            continue

                        # Convert RGBA to RGB
                        if len(rgba.shape) == 3 and rgba.shape[2] == 4:
                            rgb = rgba[:, :, :3]
                        else:
                            rgb = rgba

                        # Convert to uint8 if needed
                        if rgb.dtype != np.uint8:
                            if rgb.max() <= 1.0:
                                rgb = (rgb * 255).astype(np.uint8)
                            else:
                                rgb = rgb.astype(np.uint8)

                        # Encode as JPEG
                        import cv2
                        _, buffer = cv2.imencode('.jpg', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                                                 [cv2.IMWRITE_JPEG_QUALITY, 85])

                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

                        time.sleep(frame_delay)
                    else:
                        time.sleep(frame_delay)
                        continue

                except Exception as e:
                    print(f"Stream error for {token[:8]}: {e}")
                    import traceback
                    traceback.print_exc()
                    time.sleep(1.0)
                    continue
        finally:
            # Clean up recording mode when stream ends
            try:
                if sim and sim.camera:
                    sim.camera.pause_recording()
            except:
                pass

    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/snapshot/<token>')
def get_snapshot(token):
    """Get a single snapshot (for debugging or fallback)."""
    sim = simulations.get(token)

    if not sim or not sim._initialized or not sim.camera:
        return jsonify({'error': 'Session not found'}), 404

    try:
        # Start recording temporarily to capture frame
        was_recording = sim.camera._in_recording
        if not was_recording:
            sim.camera.start_recording()

        sim.camera.render()

        # Get frame from recorded images buffer
        if hasattr(sim.camera, '_recorded_imgs') and len(sim.camera._recorded_imgs) > 0:
            rgba = sim.camera._recorded_imgs[-1]
        else:
            return jsonify({'error': 'No frame available'}), 500

        # Stop recording if we started it
        if not was_recording:
            sim.camera.pause_recording()

        # Convert RGBA to RGB
        if len(rgba.shape) == 3 and rgba.shape[2] == 4:
            rgb = rgba[:, :, :3]
        else:
            rgb = rgba

        # Convert to uint8 if needed
        if rgb.dtype != np.uint8:
            if rgb.max() <= 1.0:
                rgb = (rgb * 255).astype(np.uint8)
            else:
                rgb = rgb.astype(np.uint8)

        import cv2
        _, buffer = cv2.imencode('.jpg', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                                 [cv2.IMWRITE_JPEG_QUALITY, 90])

        img_base64 = base64.b64encode(buffer).decode('utf-8')

        return jsonify({
            'image': img_base64,
            'timestamp': time.time()
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/health')
def health_check():
    """Health check endpoint."""
    return jsonify({
        'status': 'ok',
        'active_sessions': len([s for s in simulations.values() if s._initialized]),
        'total_sessions': len(simulations)
    })


def start_stream_server(port: int = 9003, simulations_dict: Dict = None):
    """
    Start the Flask streaming server in a background thread.

    Args:
        port: Port to run the stream server on (default: 9003)
        simulations_dict: Reference to the main server's simulations dictionary
    """
    global simulations

    if simulations_dict is not None:
        simulations = simulations_dict

    def run_server():
        # Suppress Flask's startup messages for cleaner output
        import logging
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)

        print(f"Open live view at port {port}")

        app.run(
            host='0.0.0.0',
            port=port,
            threaded=True,
            debug=False,
            use_reloader=False  # Important: disable reloader in thread
        )

    thread = threading.Thread(target=run_server, daemon=True)
    thread.start()

    return thread
