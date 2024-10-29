# ==================== import ====================

# path
from pathlib import Path

# rosbags
from rosbags.highlevel import AnyReader
from rosbags.rosbag1 import Writer
from rosbags.typesys import get_typestore, Stores
from rosbags.typesys.stores.ros1_noetic import (
    sensor_msgs__msg__CompressedImage as CompressedImage,
)

# opencv
import cv2
import numpy as np
from cv_bridge import CvBridge

# enum
from enum import Enum

# =============== helper functions ===============


def process_image(input_image):
    return cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)

def image_to_compressed_msg(image, header):
    _, compressed_image = cv2.imencode('.jpg', image)
    return CompressedImage(
        header=header,
        format='jpg',
        data=np.frombuffer(compressed_image, dtype=np.uint8),
    )


class Action(Enum):
    PASSTHROUGH = 1
    FILTER = 2


# ================== main code ==================


# helper objects
typestore = get_typestore(Stores.ROS1_NOETIC)
bridge = CvBridge()


# input bag
reader_bag_path = Path('/home/jiahao/Downloads/1710755621-2024-03-18-10-02-36-1.bag')
reader = AnyReader([reader_bag_path], default_typestore=typestore)
reader.open()

# # output bag
# writer_bag_path = Path('new.bag')
# writer = Writer(writer_bag_path)
# writer.open()


# camera topics
camera_topics = [
    '/alphasense_driver_ros/cam0/debayered/image/compressed',
    '/alphasense_driver_ros/cam1/debayered/image/compressed',
    '/alphasense_driver_ros/cam2/debayered/image/compressed'
    ]

passthrough_topics = [
    '/alphasense_driver_ros/imu',
    '/hesai/pandar'
]

other_topics_action = Action.FILTER

# # process passthrough topics and other topics
# for connection in reader.connections:

#     if connection.topic in camera_topics:
#         pass
#     elif connection.topic in passthrough_topics:
#         new_connection = writer.add_connection(connection.topic, connection.msgtype, msgdef=connection.msgdef, typestore=typestore)
#         for connection, timestamp, rawdata in reader.messages(connections=[connection]):
#             writer.write(new_connection, timestamp, rawdata)
#     else:
#         if other_topics_action == Action.PASSTHROUGH:
#             new_connection = writer.add_connection(connection.topic, connection.msgtype, msgdef=connection.msgdef, typestore=typestore)
#             for connection, timestamp, rawdata in reader.messages(connections=[connection]):
#                 writer.write(new_connection, timestamp, rawdata)
#         elif other_topics_action == Action.FILTER:
#             pass


# process camera topics

# get the three lists of messages
cam0_connection = None
cam1_connection = None
cam2_connection = None
cam0_msg_list = []
cam1_msg_list = []
cam2_msg_list = []

for connection in reader.connections:
    if connection.topic == camera_topics[0]:
        cam0_msg_list = list(reader.messages(connections=[connection]))
        cam0_connection = connection
    elif connection.topic == camera_topics[1]:
        cam1_msg_list = list(reader.messages(connections=[connection]))
        cam1_connection = connection
    elif connection.topic == camera_topics[2]:
        cam2_msg_list = list(reader.messages(connections=[connection]))
        cam2_connection = connection


# display windows
cv2.namedWindow('cam1', cv2.WINDOW_NORMAL)
cv2.namedWindow('cam0', cv2.WINDOW_NORMAL)
cv2.namedWindow('cam2', cv2.WINDOW_NORMAL)
cv2.moveWindow('cam1', 0, 0)
cv2.moveWindow('cam0', 640, 0)
cv2.moveWindow('cam2', 1280, 0)
cv2.resizeWindow('cam1', 640, 480)
cv2.resizeWindow('cam0', 640, 480)
cv2.resizeWindow('cam2', 640, 480)

current_frame = 0          
while True:
        
    # get first msg
    cam0_connection, cam0_timestamp, cam0_rawdata = cam0_msg_list[current_frame]
    cam1_connection, cam1_timestamp, cam1_rawdata = cam1_msg_list[current_frame]
    cam2_connection, cam2_timestamp, cam2_rawdata = cam2_msg_list[current_frame]

    cam0_msg = reader.deserialize(cam0_rawdata, cam0_connection.msgtype)
    cam1_msg = reader.deserialize(cam1_rawdata, cam1_connection.msgtype)
    cam2_msg = reader.deserialize(cam2_rawdata, cam2_connection.msgtype)

    cam0_image = bridge.compressed_imgmsg_to_cv2(cam0_msg, desired_encoding='passthrough')
    cam1_image = bridge.compressed_imgmsg_to_cv2(cam1_msg, desired_encoding='passthrough')
    cam2_image = bridge.compressed_imgmsg_to_cv2(cam2_msg, desired_encoding='passthrough')

    # Display the image    
    cv2.imshow('cam0', cam0_image)
    cv2.imshow('cam1', cam1_image)
    cv2.imshow('cam2', cam2_image)
    
    key = cv2.waitKey(0)
    if key == ord('q'):
        break
    elif key == 81:  # Left arrow key
        current_frame = max(0, current_frame - 1)
    elif key == 83:  # Right arrow key
        current_frame += 1
    else:
        continue

    # # modify the image to gray
    # output_image = process_image(input_image)
    
    # # compress image
    # output_msg = image_to_compressed_msg(output_image, input_msg.header)

    # # write to new bag
    # new_connection = writer.add_connection(connection.topic, message_type, msgdef=connection.msgdef, typestore=typestore)
    # new_rawdata = typestore.serialize_ros1(output_msg, message_type)
    # new_timestamp = timestamp
    # writer.write(new_connection, new_timestamp, new_rawdata)
    # break
    

# current_frame = 0
# while True:
#     connection, timestamp, rawdata = messages[current_frame]

#     msg = reader.deserialize(rawdata, connection.msgtype)

#     # read the message as image and display

#     bridge = CvBridge()

#     # Convert the ROS Image message to OpenCV format
#     cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')

#     # Display the image
#     cv2.imshow('Image', cv_image)

    

#     key = cv2.waitKey(0)
#     if key == ord('q'):
#         break
#     elif key == 81:  # Left arrow key
#         current_frame = max(0, current_frame - 1)
#     elif key == 83:  # Right arrow key
#         current_frame += 1
#     elif key == ord('s'):                        
#         # modify the image to gray
#         modified_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
#         # # Convert the image to grayscale
#         compressed_image_msg = bridge.cv2_to_compressed_imgmsg(modified_image)

#         # # write the gray image to the new bag
#         # writer.write(connection, timestamp, typestore.serialize_ros1(modified_msg, connection.msgtype))

#         modified_msg = CompressedImage(
#             msg.header,
#             format='jpeg',  # could also be 'png'
#             data=compressed_image_msg.data,
#         )

#         writer.write(
#             new_connection,
#             timestamp,
#             typestore.serialize_ros1(modified_msg, CompressedImage.__msgtype__),
#         )

        
#         print('write to bag as gray image')
#     continue


# reader.close()
# writer.close()

