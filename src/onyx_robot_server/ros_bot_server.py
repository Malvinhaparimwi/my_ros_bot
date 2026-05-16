import cv2
from flask import Flask, Response
from picamera2 import Picamera2

# Config
WIDTH = 820
HEIGHT = 640
FRAME_RATE = 30

app = Flask(__name__)

# Initialize Picamera2 (libcamera backend)
picam2 = Picamera2()

config = picam2.create_video_configuration(
    main={"size": (WIDTH, HEIGHT), "format": "RGB888"},
    controls={"FrameRate": FRAME_RATE}
)

picam2.configure(config)
picam2.start()

def generate_frames():
    while True:
        # Capture frame as NumPy array (RGB)
        frame = picam2.capture_array()

        # Convert RGB → BGR (OpenCV format)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Encode to JPEG
        success, buffer = cv2.imencode('.jpg', frame)
        if not success:
            continue

        frame_bytes = buffer.tobytes()

        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' +
            frame_bytes +
            b'\r\n'
        )

@app.route("/stream")
def camera_stream():
    return Response(
        generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=2003, threaded=True)
