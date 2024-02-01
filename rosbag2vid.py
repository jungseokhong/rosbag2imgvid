import rosbag
from cv_bridge import CvBridge
import cv2
import argparse
import os

# Set up argument parser
parser = argparse.ArgumentParser(description='Convert ROS bag images to an MP4 video.')
parser.add_argument('bag_path', help='Path to the ROS bag file')
parser.add_argument('--frame_rate', type=float, default=10.0, help='Frame rate of the output video')

# Parse arguments
args = parser.parse_args()

# Initialize CvBridge
bridge = CvBridge()

# Determine the directory and video file name
bag_name = os.path.basename(args.bag_path)
video_name = os.path.splitext(bag_name)[0] + '.mp4'

# Video writer initialization
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec definition
video_writer = None

# Open the rosbag file
with rosbag.Bag(args.bag_path, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/usb_cam/image_raw/compressed']):
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        
        if video_writer is None:
            h, w = cv_image.shape[:2]
            video_writer = cv2.VideoWriter(video_name, fourcc, args.frame_rate, (w, h))
        
        video_writer.write(cv_image)

# Release the video writer
if video_writer is not None:
    video_writer.release()

print("Video created successfully at {} FPS.".format(args.frame_rate))
