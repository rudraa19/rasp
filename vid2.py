import cv2
import numpy as np
import RPi.GPIO as GPIO

# Set up GPIO
LED_PIN = 21
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.output(LED_PIN, GPIO.LOW)  # Turn LED off initially

def count_fingers(contour):
    """
    Count the number of extended fingers based on convexity defects.
    """
    hull = cv2.convexHull(contour, returnPoints=False)
    defects = cv2.convexityDefects(contour, hull)
    if defects is None:
        return 0  # No defects detected

    finger_count = 0
    for i in range(defects.shape[0]):
        start_idx, end_idx, farthest_idx, depth = defects[i, 0]
        start = tuple(contour[start_idx][0])
        end = tuple(contour[end_idx][0])
        far = tuple(contour[farthest_idx][0])
        
        # Calculate angles to detect fingers
        a = np.linalg.norm(np.array(start) - np.array(far))
        b = np.linalg.norm(np.array(end) - np.array(far))
        c = np.linalg.norm(np.array(start) - np.array(end))
        angle = np.arccos((a**2 + b**2 - c**2) / (2 * a * b))
        
        if angle <= np.pi / 2:  # Angle < 90 degrees
            finger_count += 1

    return finger_count

# Start capturing video from the webcam
cap = cv2.VideoCapture(0)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Flip the frame horizontally for natural interaction
        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape

        # Convert the frame to HSV for skin detection
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_skin = np.array([0, 20, 70], dtype=np.uint8)
        upper_skin = np.array([20, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv_frame, lower_skin, upper_skin)

        # Clean up the mask
        mask = cv2.dilate(mask, None, iterations=2)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # Find the largest contour (assume it's the hand)
            max_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(max_contour) > 5000:  # Ignore small objects
                # Draw the contour on the frame
                cv2.drawContours(frame, [max_contour], -1, (0, 255, 0), 2)

                # Count fingers
                finger_count = count_fingers(max_contour)
                if finger_count == 4:
                    GPIO.output(LED_PIN, GPIO.HIGH)  # Turn LED ON
                    cv2.putText(frame, "4 Fingers Detected - LED ON", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                else:
                    GPIO.output(LED_PIN, GPIO.LOW)  # Turn LED OFF
                    cv2.putText(frame, f"{finger_count} Fingers Detected", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # Display the video using OpenGL or normal window
        cv2.namedWindow("Hand Gesture Tracking", cv2.WINDOW_NORMAL)
        cv2.imshow("Hand Gesture Tracking", frame)
        
        # Wait for 'q' key to break the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Exiting program...")

finally:
    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    GPIO.output(LED_PIN, GPIO.LOW)
    GPIO.cleanup()
