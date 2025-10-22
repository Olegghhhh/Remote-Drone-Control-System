# Import required libraries
import cv2  # OpenCV for computer vision tasks
import threading  # For running detection in a separate thread
from flask import Flask, Response  # Flask for web server and streaming

# Configuration constants
CAMERA_INDEX = 2  # Index of the USB camera on the TV stick
FRAME_WIDTH = 320  # Frame width for video processing
FRAME_HEIGHT = 240  # Frame height for video processing

# Initialize HOG descriptor for pedestrian detection
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

# Initialize video capture from the specified camera
cap = cv2.VideoCapture(CAMERA_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)  # Set frame width
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)  # Set frame height

# Initialize Flask application
app = Flask(__name__)
output_frame = None  # Global variable to store the processed frame
lock = threading.Lock()  # Thread lock for safe access to output_frame

def detect_people():
    """
    Detect people in video frames using HOG descriptor and draw bounding boxes.
    Runs in a separate thread to process frames continuously.
    """
    global output_frame
    while True:
        # Read a frame from the video capture
        ret, frame = cap.read()
        if not ret:
            continue  # Skip if frame capture fails

        # Resize frame for faster processing
        small_frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))

        # Detect people using HOG descriptor
        boxes, weights = hog.detectMultiScale(small_frame, winStride=(8, 8), padding=(8, 8), scale=1.05)

        # Draw bounding boxes around detected people
        for (x, y, w, h) in boxes:
            cv2.rectangle(small_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Store the processed frame safely using a lock
        with lock:
            output_frame = small_frame.copy()

def generate():
    """
    Generate a stream of JPEG frames for the web interface.
    Yields frames in a multipart response format.
    """
    global output_frame
    while True:
        # Access the processed frame safely
        with lock:
            if output_frame is None:
                continue  # Skip if no frame is available
            # Encode frame as JPEG
            ret, buffer = cv2.imencode('.jpg', output_frame)
            frame = buffer.tobytes()
        # Yield frame in multipart/x-mixed-replace format
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    """
    Flask route to stream video frames to the client.
    Returns a Response object with the streaming generator.
    """
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    # Start the people detection in a separate thread
    t = threading.Thread(target=detect_people)
    t.daemon = True  # Set as daemon so it terminates when the main program exits
    t.start()

    # Run the Flask web server
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
