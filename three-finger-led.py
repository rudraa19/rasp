import cv2
import mediapipe as mp
import RPi.GPIO as GPIO
import time

# Setup GPIO for LEDs
LED_PINS = {1: 21, 2: 22, 3: 23}  # Map finger count to respective LED pins
GPIO.setmode(GPIO.BCM)

# Set up all LED pins
for pin in LED_PINS.values():
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

# Initialize Mediapipe Hand Detection
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7)
mp_drawing = mp.solutions.drawing_utils

# Initialize webcam
camera = cv2.VideoCapture(0)

def count_fingers(hand_landmarks):
    """
    Count the number of fingers that are up.
    """
    tips = [4, 8, 12, 16, 20]  # Thumb, Index, Middle, Ring, Pinky tips
    up_fingers = 0

    # Thumb
    if hand_landmarks.landmark[tips[0]].x < hand_landmarks.landmark[tips[0] - 2].x:
        up_fingers += 1

    # Other fingers
    for tip in tips[1:]:
        if hand_landmarks.landmark[tip].y < hand_landmarks.landmark[tip - 2].y:
            up_fingers += 1

    return up_fingers

def control_leds(fingers):
    """
    Light up the corresponding LED based on the number of fingers.
    """
    # Turn off all LEDs first
    for pin in LED_PINS.values():
        GPIO.output(pin, GPIO.LOW)

    # Turn on the LED corresponding to the finger count
    if fingers in LED_PINS:
        GPIO.output(LED_PINS[fingers], GPIO.HIGH)

try:
    while True:
        ret, frame = camera.read()
        if not ret:
            print("Failed to grab frame.")
            break

        # Flip the frame horizontally for a mirror-like effect
        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Process the frame with Mediapipe
        results = hands.process(rgb_frame)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw hand landmarks on the frame
                mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                # Count fingers
                fingers = count_fingers(hand_landmarks)
                print(f"Fingers up: {fingers}")

                # Control LEDs based on finger count
                control_leds(fingers)

        # Display the frame
        cv2.imshow("Hand Tracking", frame)

        # Break loop with 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Exiting program...")

finally:
    # Cleanup
    camera.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()
