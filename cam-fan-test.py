import cv2
import mediapipe as mp
import RPi.GPIO as GPIO
import time

# ================= GPIO SETUP =================
RELAY_PIN = 21
GPIO.setmode(GPIO.BCM)
GPIO.setup(RELAY_PIN, GPIO.OUT)
GPIO.output(RELAY_PIN, GPIO.LOW)

relay_state = False  # False = OFF, True = ON
finger_was_up = False  # For edge detection

# ================= MEDIAPIPE SETUP =================
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7
)
mp_draw = mp.solutions.drawing_utils

# ================= CAMERA =================
cap = cv2.VideoCapture(0)

def is_index_finger_up(hand_landmarks):
    """
    Returns True if index finger is up.
    Ignores all other fingers.
    """
    index_tip = hand_landmarks.landmark[8]
    index_pip = hand_landmarks.landmark[6]

    return index_tip.y < index_pip.y  # finger up

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        result = hands.process(rgb)

        finger_up = False

        if result.multi_hand_landmarks:
            hand_landmarks = result.multi_hand_landmarks[0]
            mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            finger_up = is_index_finger_up(hand_landmarks)

        # ================= TOGGLE LOGIC =================
        if finger_up and not finger_was_up:
            relay_state = not relay_state
            GPIO.output(RELAY_PIN, GPIO.HIGH if relay_state else GPIO.LOW)
            print(f"Relay toggled -> {'ON' if relay_state else 'OFF'}")
            time.sleep(0.3)  # debounce delay

        finger_was_up = finger_up

        cv2.imshow("Index Finger Toggle", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    pass

finally:
    cap.release()
    cv2.destroyAllWindows()
    GPIO.output(RELAY_PIN, GPIO.LOW)
    GPIO.cleanup()
