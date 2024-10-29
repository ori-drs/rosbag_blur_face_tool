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



bagpath = Path('/home/jiahao/Downloads/1710755621-2024-03-18-10-02-36-1.bag')
new_bagpath = Path('new.bag')

# bagpath = Path('new.bag')
# new_bagpath = Path('new2.bag')

camera_topics = [
    '/alphasense_driver_ros/cam0/debayered/image/compressed',
    # '/alphasense_driver_ros/cam1/debayered/image/compressed',
    # '/alphasense_driver_ros/cam2/debayered/image/compressed'
    ]


# Create a type store to use if the bag has no message definitions.
typestore = get_typestore(Stores.ROS1_NOETIC)


reader = AnyReader([bagpath], default_typestore=typestore)
writer = Writer(new_bagpath)
reader.open()
writer.open()

bridge = CvBridge()

for connection in reader.connections:

    # skip if not in camera topics
    if connection.topic not in camera_topics:
        continue
    
    # if is camera topics
    new_connection = writer.add_connection(connection.topic, connection.msgtype, typestore=typestore)
    input_msg_list = list(reader.messages(connections=[connection]))

    # get first msg
    connection, timestamp, rawdata = input_msg_list[0]
    input_msg = reader.deserialize(rawdata, connection.msgtype)


    # read image from msg
    input_image = bridge.compressed_imgmsg_to_cv2(input_msg, desired_encoding='passthrough')

    # Display the image
    cv2.imshow('Image', input_image)
    cv2.waitKey(0)

    # modify the image to gray
    output_image = cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)
    
    
    # # write the gray image to the new bag
    # writer.write(connection, timestamp, typestore.serialize_ros1(modified_msg, connection.msgtype))

    _, compressed_image = cv2.imencode('.jpg', output_image)
    output_msg = CompressedImage(
        input_msg.header,
        format='jpeg',
        data=np.frombuffer(compressed_image, dtype=np.uint8),
    )

    writer.write(
        new_connection,
        timestamp,
        typestore.serialize_ros1(output_msg, connection.msgtype),
    )
    
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

        

        # key = cv2.waitKey(0)
        # if key == ord('q'):
        #     break
        # elif key == 81:  # Left arrow key
        #     current_frame = max(0, current_frame - 1)
        # elif key == 83:  # Right arrow key
        #     current_frame += 1
        # elif key == ord('s'):                        
        #     # modify the image to gray
        #     modified_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
        #     # # Convert the image to grayscale
        #     compressed_image_msg = bridge.cv2_to_compressed_imgmsg(modified_image)

        #     # # write the gray image to the new bag
        #     # writer.write(connection, timestamp, typestore.serialize_ros1(modified_msg, connection.msgtype))

        #     modified_msg = CompressedImage(
        #         msg.header,
        #         format='jpeg',  # could also be 'png'
        #         data=compressed_image_msg.data,
        #     )

        #     writer.write(
        #         new_connection,
        #         timestamp,
        #         typestore.serialize_ros1(modified_msg, CompressedImage.__msgtype__),
        #     )

            
        #     print('write to bag as gray image')
        # continue


reader.close()
writer.close()

