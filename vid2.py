import cv2
import numpy as np
import RPi.GPIO as GPIO

# Initialize GPIO for LED
LED_PIN = 21
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.output(LED_PIN, GPIO.LOW)

def detect_fingers(frame):
    """
    Detect the number of fingers raised in the given frame.
    Args:
        frame (numpy array): The input video frame.
    Returns:
        int: The number of fingers detected.
    """
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian Blur
    blur = cv2.GaussianBlur(gray, (35, 35), 0)

    # Thresholding
    _, thresh = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    # Find contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Get the largest contour (assume it's the hand)
        max_contour = max(contours, key=cv2.contourArea)

        # Find convex hull
        hull = cv2.convexHull(max_contour, returnPoints=False)

        # Find convexity defects
        defects = cv2.convexityDefects(max_contour, hull)

        if defects is not None:
            # Count fingers
            finger_count = 0
            for i in range(defects.shape[0]):
                s, e, f, d = defects[i, 0]
                start = tuple(max_contour[s][0])
                end = tuple(max_contour[e][0])
                far = tuple(max_contour[f][0])

                # Calculate angles to count fingers
                a = np.linalg.norm(np.array(start) - np.array(far))
                b = np.linalg.norm(np.array(end) - np.array(far))
                c = np.linalg.norm(np.array(start) - np.array(end))
                angle = np.arccos((a**2 + b**2 - c**2) / (2 * a * b))  # Cosine rule

                # Angle threshold to detect fingers
                if angle <= np.pi / 2:  # Angle less than 90 degrees
                    finger_count += 1
                    cv2.circle(frame, far, 4, [0, 0, 255], -1)

            # Return the number of fingers
            return finger_count + 1  # +1 for the thumb
    return 0

def main():
    """
    Main function to capture video feed, detect fingers, and control LED.
    """
    # Open camera feed
    cap = cv2.VideoCapture(0)
    cap.set(3, 640)  # Set width
    cap.set(4, 480)  # Set height

    if not cap.isOpened():
        print("Error: Could not open video stream.")
        return

    print("Press 'q' to quit.")
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Could not read frame.")
                break

            # Flip the frame horizontally (mirror view)
            frame = cv2.flip(frame, 1)

            # Detect fingers
            finger_count = detect_fingers(frame)

            # Turn on LED if four fingers are detected
            if finger_count == 4:
                GPIO.output(LED_PIN, GPIO.HIGH)
                cv2.putText(frame, "LED ON: Four Fingers Detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                GPIO.output(LED_PIN, GPIO.LOW)
                cv2.putText(frame, f"Fingers: {finger_count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Show the frame
            cv2.imshow("Hand Tracking", frame)

            # Break loop on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        print("\nProgram interrupted.")
    finally:
        # Cleanup
        cap.release()
        cv2.destroyAllWindows()
        GPIO.output(LED_PIN, GPIO.LOW)
        GPIO.cleanup()
        print("Program terminated.")

if __name__ == "__main__":
    main()
