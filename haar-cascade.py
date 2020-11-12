import cv2
import sys

cascPaths = sys.argv[1:]
cascadeClassifiers = [cv2.CascadeClassifier(cascPath) for cascPath in cascPaths]

video_capture = cv2.VideoCapture(0)

while True:
    # Capture frame-by-frame
    ret, frame = video_capture.read()

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

    # Draw a rectangle around the faces
    for i, v in enumerate(detected):
        (x, y, w, h) = v
        red = int(bool( (1 << 0) & classifier_labels[i] )) * 255
        green = int(bool( (1 << 1) & classifier_labels[i] )) * 255
        blue = int(bool( (1 << 2) & classifier_labels[i] )) * 255
        color = (red, green, blue)
        cv2.rectangle(frame, (x, y), (x+w, y+h), color, 2)

    # Display the resulting frame
    cv2.imshow('Video', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
