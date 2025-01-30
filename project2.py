import cv2
import RPi.GPIO as GPIO
import time
from cvzone.HandTrackingModule import HandDetector
import os
import threading

os.environ["OPENCV_VIDEOIO_DEBUG"] = "1"
os.environ["OPENCV_VIDEOIO_PRIORITY_MSMF"] = "0"
#os.system("echo 0 | sudo tee /sys/module/usbcore/parameters/autosuspend")

relay_status_map = [0, 0]
relay_cycle_map = [0, 0]
relay_pin_map = [21, 10]

LED_PIN = 4
"""
-1 = none
0 = reading
1 = cam_err
"""
LED_PIN_STATE = 0

GPIO.setmode(GPIO.BCM)

def setup_all_pins():
	for pin in relay_pin_map:
		print(f"Setting {pin} as OUT")
		GPIO.setup(pin, GPIO.OUT)
	GPIO.setup(LED_PIN, GPIO.OUT)

def set_all_pins_low():
	for pin in relay_pin_map:
		print(f"{pin}: LOW")
		GPIO.output(pin, GPIO.LOW)
	GPIO.output(LED_PIN, GPIO.LOW)

setup_all_pins()
set_all_pins_low()

def blink_led():
	while True:
		print(f"LED_PIN_STATE: {LED_PIN_STATE}")
		if LED_PIN_STATE == 0:
			print("LED HIGH")
			GPIO.output(LED_PIN, GPIO.HIGH)
			time.sleep(0.1)
			print("LED LOW")
			GPIO.output(LED_PIN, GPIO.LOW)
			time.sleep(1)
		if LED_PIN_STATE == 1:
			print("LED HIGH")
			GPIO.output(LED_PIN, GPIO.HIGH)
			time.sleep(0.1)
			print("LED LOW")
			GPIO.output(LED_PIN, GPIO.LOW)
			time.sleep(0.1)
			print("LED HIGH")
			GPIO.output(LED_PIN, GPIO.HIGH)
			time.sleep(0.1)
			print("LED LOW")
			GPIO.output(LED_PIN, GPIO.LOW)


infinite_led_blink_thread = threading.Thread(target=blink_led)
infinite_led_blink_thread.start()

# Initialize webcam
camera = cv2.VideoCapture("/dev/webcam")
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

def set_relay_cycle(relay_number, cycle_number):
	relay_cycle_map[relay_number] = cycle_number

def get_relay_cycle(relay_number):
	return relay_cycle_map[relay_number]

def set_relay(relay_number, value):
	relay_status_map[relay_number] = value

def handle_fingerup_cycle(fingerup, fingerIndex, relay_number):
	if fingerup[fingerIndex] == 1:
		if get_relay_cycle(relay_number) == 0:
			set_relay_cycle(relay_number, 1)
			set_relay(relay_number, 1)
			time.sleep(0.1)
		elif get_relay_cycle(relay_number) == 2:
			set_relay_cycle(relay_number, 3)
			set_relay(relay_number, 0)
			time.sleep(0.1)

detector = HandDetector(maxHands=1, detectionCon=0.9)

def cleanup():
	print("Cleaning up...")
	camera.release()
	cv2.destroyAllWindows()
	set_all_pins_low()


PIN_OUTPUT_MAP = {}

def set_pin_output(pin, state):
	if pin in PIN_OUTPUT_MAP:
		if PIN_OUTPUT_MAP[pin] == state:
			return
	PIN_OUTPUT_MAP[pin] = state
	GPIO.output(pin, state)
	time.sleep(1)

try:
	while True:

		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
		
		LED_PIN_STATE = 1
		ret, frame = camera.read()
		if not ret:
			print("Failed to grab frame")
			LED_PIN_STATE = 1
			time.sleep(1)
			camera.release()
			#time.sleep(1)  # Short delay before reinitializing
			camera = cv2.VideoCapture("/dev/webcam")
			continue

		if LED_PIN_STATE == 1:
			LED_PIN_STATE = 0

		# Flip the frame horizontally for a mirror-like effect
		frame = cv2.flip(frame, 1)
		rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

		# Process the frame with Mediapipe
		hand, frame = detector.findHands(frame, draw=True)

		# print(f"cycle_map: {relay_cycle_map}")
		if hand:
			hand_info = hand[0]

			if hand_info:
				fingerup = detector.fingersUp(hand_info)
				print(f"fingerup: {fingerup}")

				handle_fingerup_cycle(fingerup, 1, 0)
				handle_fingerup_cycle(fingerup, 4, 1)
		else:
			for index, _ in enumerate(relay_status_map):
				# print(f"index: {index}")
				# print(f"relay_cycle_map: {relay_cycle_map}")
				if get_relay_cycle(index) == 1:
					set_relay_cycle(index, 2)
				elif get_relay_cycle(index) == 3:
					set_relay_cycle(index, 0)

		# print(f"relay_status_map: {relay_status_map}")
		for index, relay_status in enumerate(relay_status_map):
			#print(f"index: {index}")
			pin = relay_pin_map[index]
			if relay_status == 1:
				print(f"PIN: {pin} -> HIGH")
				set_pin_output(pin, GPIO.HIGH)
			else:
				print(f"PIN: {pin} -> LOW")
				set_pin_output(pin, GPIO.LOW)

		# Display the frame
		# cv2.imshow("Hand Tracking", frame)

except KeyboardInterrupt:
	print("Exiting program...")

finally:
	cleanup()
