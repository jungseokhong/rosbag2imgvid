import rosbag
from cv_bridge import CvBridge
import cv2
import argparse
import os

# Set up argument parser
parser = argparse.ArgumentParser(description='Extract images from a ROS bag.')
parser.add_argument('bag_path', help='Path to the ROS bag file')

# Parse arguments
args = parser.parse_args()

# Initialize CvBridge
bridge = CvBridge()

# Create a directory named after the bag file
bag_name = os.path.basename(args.bag_path)
dir_name = os.path.splitext(bag_name)[0]
if not os.path.exists(dir_name):
    os.makedirs(dir_name)

# Open the rosbag file
with rosbag.Bag(args.bag_path, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/usb_cam/image_raw/compressed']):
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        
        # Define the filename based on ROS timestamp for uniqueness
        filename = os.path.join(dir_name, 'image_{}.jpg'.format(t.to_nsec()))
        
        # Save the image
        cv2.imwrite(filename, cv_image)

print("Images extracted successfully.")
