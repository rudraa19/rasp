import cv2

# Initialize the webcam (default device index is 0, change if needed)
camera = cv2.VideoCapture(0)

# Check if the webcam is opened successfully
if not camera.isOpened():
    print("Error: Could not open webcam.")
    exit()

# Capture a single frame
ret, frame = camera.read()

if ret:
    # Save the captured frame to a file
    image_path = "captured_image.jpg"
    cv2.imwrite(image_path, frame)
    print(f"Image captured and saved to {image_path}")
else:
    print("Error: Could not capture image.")

# Release the webcam
camera.release()
