import cv2
import sys
import serial
from time import sleep
import picamera
import numpy as np

ser = serial.Serial("/dev/ttyS0", 115200)
received_data = b"short message\0"

cascPaths = sys.argv[1:]
cascadeClassifiers = [cv2.CascadeClassifier(cascPath) for cascPath in cascPaths]

# video_capture = cv2.VideoCapture(0)

while True:
    # Capture frame-by-frame
    # ret, frame = video_capture.read()
    with picamera.PiCamera() as camera:
        camera.resolution = (640, 480)
        camera.framerate = 24
        frame = np.empty((480, 640, 3), dtype = np.uint8)
        camera.capture(frame, 'rgb')

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    detected = []
    classifier_labels = []
    classifier_label = 0
    for cascadeClassifier in cascadeClassifiers:
        new_detected = cascadeClassifier.detectMultiScale(
                            gray,
                            scaleFactor=1.1,
                            minNeighbors=5,
                            minSize=(30, 30),
                            flags=cv2.CASCADE_SCALE_IMAGE # cv2.CV_HAAR_SCALE_IMAGE
                        )
        classifier_labels.extend([classifier_label]*len(new_detected))
        classifier_label += 1
        detected.extend(new_detected)

    face_detected = False
    # Draw a rectangle around the faces
    for i, v in enumerate(detected):
        (x, y, w, h) = v
        red = int(bool( (1 << 0) & classifier_labels[i] )) * 255
        green = int(bool( (1 << 1) & classifier_labels[i] )) * 255
        blue = int(bool( (1 << 2) & classifier_labels[i] )) * 255
        color = (red, green, blue)
        cv2.rectangle(frame, (x, y), (x+w, y+h), color, 2)

        # serial output
        serial_value = f"F{int(x/frame.shape[1]*255):03d}{int((x+w)/frame.shape[1]*255):03d}"
        print(serial_value)
        serial_data = serial_value.encode('utf-8') + b"\0"
        ser.write(serial_data)

        # wait for response
        print("waiting for response")
        received_data = ser.read()
        sleep(0.03)
        data_left = ser.inWaiting()
        received_data += ser.read(data_left)
        print("response received")

        face_detected = True
        sleep(0.5)

    # Display the resulting frame
    # cv2.imshow('Video', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    if not face_detected:
        sleep(0.2)
