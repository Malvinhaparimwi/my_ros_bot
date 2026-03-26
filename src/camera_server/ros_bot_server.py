import cv2
from cv_bridge import CvBridge
from flask import Flask, Response

 # Declare constants for flexibility
DEVICE_ID = 4
FRAME_RATE = 30.0
WIDTH = 820
HEIGHT = 640

# Initalize the flask app
app = Flask(__name__)

# Initialize OpenCV camera
cap = cv2.VideoCapture(DEVICE_ID)

# Generating frames
def generate_frames():
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame from camera.")
            break

        # Resize the frame (optional)
        frame = cv2.resize(frame, (WIDTH, HEIGHT), interpolation=cv2.INTER_CUBIC)

        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()

        yield(b'--frame\r\n'
              b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        
@app.route("/stream")
def camera_stream():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=2003)