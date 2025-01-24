
import os

def record_video(output_file="video.h264", width=640, height=480, fps=30, duration=30):
    """
    Record a video using Raspberry Pi's libcamera.
    
    Args:
        output_file (str): The name of the output video file.
        width (int): Video width (default 640).
        height (int): Video height (default 480).
        fps (int): Frames per second (default 30).
        duration (int): Duration in seconds (default 30).
    """
    # Command to record a video
    command = f"libcamera-vid -o {output_file} --width {width} --height {height} --framerate {fps} -t {duration * 1000}"

    # Execute the command
    print("Recording video...")
    result = os.system(command)

    if result == 0:
        print(f"Video saved as {output_file}")
    else:
        print("Failed to record video. Make sure the camera is connected and enabled.")

# Record a video with the specified settings
record_video("output_video.h264", width=640, height=480, fps=30, duration=30)
