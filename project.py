import cv2
import RPi.GPIO as GPIO
import time
from cvzone.HandTrackingModule import HandDetector
import os

os.environ["OPENCV_VIDEOIO_DEBUG"] = "1"
os.environ["OPENCV_VIDEOIO_PRIORITY_MSMF"] = "0"
#os.system("echo 0 | sudo tee /sys/module/usbcore/parameters/autosuspend")

relay_status_map = [0]
relay_cycle_map = [0]
relay_pin_map = [21]

GPIO.setmode(GPIO.BCM)

def set_all_pins_low():
	for pin in relay_pin_map:
		GPIO.output(pin, GPIO.LOW)

for pin in relay_pin_map:
	GPIO.setup(pin, GPIO.OUT)

set_all_pins_low()

# Initialize webcam
camera = cv2.VideoCapture("/dev/webcam")

def set_relay_cycle(relay_number, cycle_number):
	relay_cycle_map[relay_number] = cycle_number

def get_relay_cycle(relay_number):
	return relay_cycle_map[relay_number]

def set_relay(relay_number, value):
	relay_status_map[relay_number] = value

detector = HandDetector(maxHands=1, detectionCon=0.8)

def cleanup():
	print("Cleaning up...")
	camera.release()
	cv2.destroyAllWindows()
	set_all_pins_low()

try:
	while True:
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

		ret, frame = camera.read()
		if not ret:
			camera.release()
			#camera = None
			time.sleep(1)  # Short delay before reinitializing
			camera = cv2.VideoCapture("/dev/webcam")
			continue

		# Flip the frame horizontally for a mirror-like effect
		frame = cv2.flip(frame, 1)
		rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

		# Process the frame with Mediapipe
		hand, frame = detector.findHands(frame, draw=True)

		print(f"cycle_map: {relay_cycle_map}")
		if hand:
			hand_info = hand[0]

			if hand_info:
				fingerup = detector.fingersUp(hand_info)
				print(f"fingerup: {fingerup}")

				if fingerup[1] == 1:
					if get_relay_cycle(0) == 0:
						set_relay_cycle(0, 1)
						set_relay(0, 1)
						time.sleep(0.1)
					elif get_relay_cycle(0) == 2:
						set_relay_cycle(0, 3)
						set_relay(0, 0)
						time.sleep(0.1)
		else:
			if get_relay_cycle(0) == 1:
				set_relay_cycle(0, 2)
			elif get_relay_cycle(0) == 3:
				set_relay_cycle(0, 0)

		print(f"relay_status_map: {relay_status_map}")
		for index, relay_status in enumerate(relay_status_map):
			#print(f"index: {index}")
			pin = relay_pin_map[index]
			if relay_status == 1:
				print(f"PIN: {pin} -> HIGH")
				GPIO.output(pin, GPIO.HIGH)
			else:
				print(f"PIN: {pin} -> LOW")
				GPIO.output(pin, GPIO.LOW)

		# Display the frame
		# cv2.imshow("Hand Tracking", frame)

except KeyboardInterrupt:
	print("Exiting program...")

finally:
	cleanup()