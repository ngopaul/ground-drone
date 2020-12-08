# import the necessary packages
from collections import deque
from imutils.video import VideoStream
import cv2
import imutils
import time
import serial
import os
import sys
from flask import Flask, render_template, Response, request
from filelock import Timeout, FileLock

if sys.platform == 'linux' and os.uname().nodename == 'raspberrypi':
    ser = serial.Serial("/dev/ttyS0", 115200)
app = Flask(__name__)
lock = FileLock("lock.txt.lock", timeout=1)

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
greenLower = (60, 110, 10)
greenUpper = (95, 255, 200)
# if a video path was not supplied, grab the reference
# to the webcam
vs = VideoStream(src=0).start()
# allow the camera to warm up
time.sleep(2.0)


def process_frame(frame, follow=False):
    # resize the frame, blur it, and convert it to the HSV
    # color space
    global mode
    frame = imutils.resize(frame, width=600)
    if not follow:
        return frame
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None
    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # only proceed if the radius meets a minimum size
        if radius > 40:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius),
                       (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

    if len(cnts) > 0 and radius > 40:
        left_bound = max(0, x - radius)
        right_bound = min(600, x + radius)
        left_percent = left_bound / 600 * 100
        right_percent = right_bound / 600 * 100

        serial_value = f"F{int(left_percent):03d}{int(right_percent):03d}"
        print(serial_value)
        serial_data = serial_value.encode('utf-8') + b"\0"
        if sys.platform == 'linux' and os.uname().nodename == 'raspberrypi':
            lock.acquire()
            try:
                # open(file_path, "a").write(serial_value)
                ser.write(serial_data)
            finally:
                time.sleep(1)
                lock.release()
#            ser.write(serial_data)
    return frame


def gen_frames(follow=False):  # generate frame by frame from camera
    while True:
        # Capture frame-by-frame
        start_frame_time = int(round(time.time() * 1000))
        frame = vs.read()  # read the camera frame
        frame = process_frame(frame, follow)
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        end_frame_time = int(round(time.time() * 1000))
        # ensure 10 fps
        time.sleep(max(0, int(1 / 10 - (end_frame_time - start_frame_time) / 1000)))
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')  # concat frame one by one and show result


@app.route('/video_feed')
def video_feed():
    # Video streaming route. Put this in the src attribute of an img tag
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/video_feed_follow')
def video_feed_follow():
    # Video streaming route. Put this in the src attribute of an img tag
    return Response(gen_frames(follow=True), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/')
@app.route('/index.html')
def index():
    """Follower control mode 0"""
    return render_template('index.html')


@app.route('/user-control.html')
def user_page():
    """User Control mode 1"""
    return render_template('user-control.html')


@app.route('/postuser', methods=['POST'])
def post_user_control():
    with lock:
        serial_value = request.form['serial_value']
        print(serial_value)
        if serial_value[0] == 'U' and len(serial_value) == 5:
            serial_data = serial_value.encode('utf-8') + b"\0"
            if sys.platform == 'linux' and os.uname().nodename == 'raspberrypi':
                ser.write(serial_data)
                time.sleep(0.5)
    return ""


if __name__ == '__main__':
    app.run(host='0.0.0.0', threaded=True)
