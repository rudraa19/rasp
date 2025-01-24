
import os

def take_picture(output_file="image.jpg"):
    """
    Capture an image using Raspberry Pi's libcamera.
    """
    # Command to take a picture
    command = f"libcamera-jpeg -o {output_file} -n"

    # Execute the command
    print("Capturing image...")
    result = os.system(command)

    if result == 0:
        print(f"Image saved as {output_file}")
    else:
        print("Failed to capture image. Make sure the camera is connected and enabled.")

# Take a picture and save it as 'output.jpg'
take_picture("output.jpg")
