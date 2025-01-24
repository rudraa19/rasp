import cv2
import mediapipe as mp
import RPi.GPIO as GPIO
import time

# Set up GPIO
LED_PIN = 21
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.output(LED_PIN, GPIO.LOW)  # Turn LED off initially

# Mediapipe Hand Detection
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

# Initialize the hand detector
hands = mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7)

def count_fingers(hand_landmarks):
    """
    Count the number of extended fingers based on landmarks.
    """
    fingers = []
    
    # Tips of fingers (index: 4 for thumb, 8 for index, 12 for middle, 16 for ring, 20 for pinky)
    finger_tips = [4, 8, 12, 16, 20]
    
    # Thumb
    if hand_landmarks.landmark[finger_tips[0]].x < hand_landmarks.landmark[finger_tips[0] - 2].x:
        fingers.append(1)
    else:
        fingers.append(0)

    # Four fingers
    for tip in finger_tips[1:]:
        if hand_landmarks.landmark[tip].y < hand_landmarks.landmark[tip - 2].y:
            fingers.append(1)
        else:
            fingers.append(0)
    
    return fingers.count(1)

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
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Process the frame with Mediapipe
        results = hands.process(rgb_frame)
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw hand landmarks on the frame
                mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                
                # Count fingers
                fingers = count_fingers(hand_landmarks)
                if fingers == 4:
                    GPIO.output(LED_PIN, GPIO.HIGH)  # Turn LED ON
                    cv2.putText(frame, "4 Fingers Detected - LED ON", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                else:
                    GPIO.output(LED_PIN, GPIO.LOW)  # Turn LED OFF
                    cv2.putText(frame, f"{fingers} Fingers Detected", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        # Show the frame
        cv2.imshow("Hand Gesture Recognition", frame)
        
        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Exiting program...")

finally:
    # Clean up
    cap.release()
    cv2.destroyAllWindows()
    hands.close()
    GPIO.output(LED_PIN, GPIO.LOW)
    GPIO.cleanup()
