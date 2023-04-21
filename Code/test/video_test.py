import cv2
import numpy as np

# Define the codec and create a VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
fps = 30.0
frame_size = (640, 480) # (width, height)
out = cv2.VideoWriter('output.mp4', fourcc, fps, frame_size)

# Create a single grayscale frame with a white background and a black square in the center
frame = np.ones((frame_size[1], frame_size[0]), dtype=np.uint8) * 255
square_size = min(frame_size) // 2 - 10
square_x = (frame_size[0] - square_size) // 2
square_y = (frame_size[1] - square_size) // 2
frame[square_y:square_y+square_size, square_x:square_x+square_size] = 0

# Convert the grayscale frame to a 3-channel RGB frame for writing to the video
rgb_frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)

# Write the frame multiple times to create the video
num_frames = int(fps * 3)  # 3 seconds of video
for i in range(num_frames):
    out.write(rgb_frame)

# Release the video writer and close the file
out.release()
