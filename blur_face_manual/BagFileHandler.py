# pathlib
from pathlib import Path

# numpy
import numpy as np

# enum
from enum import Enum

# cv2
from cv_bridge import CvBridge
import cv2

# rosbags
from rosbags.typesys import get_typestore, Stores
from rosbags.highlevel import AnyReader, AnyReaderError
from rosbags.rosbag1 import Writer, WriterError
from rosbags.typesys.stores.ros1_noetic import sensor_msgs__msg__CompressedImage as CompressedImage

# blur_face_manual
from blur_face_manual.Cam import Cam
from blur_face_manual.BlurRegion import blur_image

class Action(Enum):
    PASSTHROUGH = 1
    FILTER = 2

class BagFileHandler:
    def __init__(self, path: str):
        self.input_bag_path = path
        self.typestore = get_typestore(Stores.ROS1_NOETIC)
        self.bridge = CvBridge()
        
        self.passthrough_topics = [
            '/alphasense_driver_ros/imu',
            '/hesai/pandar'
        ]

        self.other_topics_action = Action.FILTER

    def get_cams(self):
        # cam topics
        cam0_topic = '/alphasense_driver_ros/cam0/debayered/image/compressed'
        cam1_topic = '/alphasense_driver_ros/cam1/debayered/image/compressed'
        cam2_topic = '/alphasense_driver_ros/cam2/debayered/image/compressed'
        cam_topics = [cam0_topic, cam1_topic, cam2_topic]

        # read image from bag
        reader = self.create_reader(self.input_bag_path)
        if reader:
            reader.open()
            cams = self.read_images_from_bag(reader, cam_topics)
            reader.close()
        else:
            print('Quitting.')
            exit(1)
        
        return cams

    def image_to_compressed_msg(self, image, header):
        _, compressed_image = cv2.imencode('.jpg', image)
        return CompressedImage(
            header=header,
            format='jpg',
            data=np.frombuffer(compressed_image, dtype=np.uint8),
        )
    

    def create_reader(self, path):
        try:
            reader = AnyReader([path], default_typestore=self.typestore)
            return reader
        except AnyReaderError as e:
            print(f'Cannot open bag file "{path}".')
            return None
        except Exception as e:
            print('An error occurred while opening the bag file.')
            print (e)
            return None

    def create_writer(self, path):
        # create bag file
        try:
            writer = Writer(path)
            return writer
        except WriterError as e:
            print('Bag already exists, please rename or delete the existing bag and try again.')
            return None
        except Exception as e:
            print('An error occurred while opening the bag file.')
            return None

    def write_images_to_bag(self, writer, cams):
        # for each camera
        for ith, cam in enumerate(cams):
            input_connection, _, _ = cam.msg_list[0]
            output_connection = writer.add_connection(input_connection.topic, input_connection.msgtype, msgdef=input_connection.msgdef, typestore=self.typestore)  

            # write for each frame
            for frame in range(cam.total_frames):
                print (f'Writing frame {frame} for cam {ith}')
                input_connection, input_timestamp, input_rawdata = cam.msg_list[frame]

                # check if new blur regions are added
                if cam.blur_regions[frame]:
                    input_msg = self.typestore.deserialize_ros1(input_rawdata, input_connection.msgtype)
                    input_image = self.bridge.compressed_imgmsg_to_cv2(input_msg, desired_encoding='passthrough')

                    output_image = input_image.copy()
                    blur_image(output_image, cam.blur_regions[frame])

                    output_timestamp = input_timestamp
                    output_msg = self.image_to_compressed_msg(output_image, input_msg.header)
                    output_rawdata = self.typestore.serialize_ros1(output_msg, input_connection.msgtype)
                    writer.write(output_connection, output_timestamp, output_rawdata)
                else:
                    writer.write(output_connection, input_timestamp, input_rawdata)

    def write_other_topics_to_bag(self, reader, writer):
        # process passthrough topics and other topics
        for input_connection in reader.connections:
            if input_connection.topic in self.passthrough_topics:
                output_connection = writer.add_connection(input_connection.topic, input_connection.msgtype, msgdef=input_connection.msgdef, typestore=self.typestore)
                for input_connection, timestamp, rawdata in reader.messages(connections=[input_connection]):
                    print(f'Writing message of timestamp {timestamp} for topic {input_connection.topic}')
                    writer.write(output_connection, timestamp, rawdata)
            else:
                if self.other_topics_action == Action.PASSTHROUGH:
                    output_connection = writer.add_connection(input_connection.topic, input_connection.msgtype, msgdef=input_connection.msgdef, typestore=self.typestore)
                    for input_connection, timestamp, rawdata in reader.messages(connections=[input_connection]):
                        print(f'Writing message of timestamp {timestamp} for topic {input_connection.topic}')
                        writer.write(output_connection, timestamp, rawdata)
                elif self.other_topics_action == Action.FILTER:
                    pass

    def export_data_to_bag(self, output_bag_name, cams):                
        new_path = Path(output_bag_name)

        writer = self.create_writer(new_path)
        reader = self.create_reader(self.input_bag_path)
        if writer and reader:
            writer.open()
            reader.open()

            self.write_images_to_bag(writer, cams)
            self.write_other_topics_to_bag(reader, writer)
            
            writer.close()
            reader.close()
            print(f'Bag file written to {writer.path}')
        else:
            return
        

    def process_passthrough_and_other_topics(self):
        # process passthrough topics and other topics
        for connection in self.reader.connections:

            if connection.topic in self.cam_topics:
                pass
            elif connection.topic in self.passthrough_topics:
                new_connection = self.writer.add_connection(connection.topic, connection.msgtype, msgdef=connection.msgdef, typestore=self.typestore)
                for connection, timestamp, rawdata in self.reader.messages(connections=[connection]):
                    self.writer.write(new_connection, timestamp, rawdata)
            else:
                if self.other_topics_action == Action.PASSTHROUGH:
                    new_connection = self.writer.add_connection(connection.topic, connection.msgtype, msgdef=connection.msgdef, typestore=self.typestore)
                    for connection, timestamp, rawdata in self.reader.messages(connections=[connection]):
                        self.writer.write(new_connection, timestamp, rawdata)
                elif self.other_topics_action == Action.FILTER:
                    pass

    def read_images_from_bag(self, reader, cam_topics):     
        cams = [Cam() for _ in range(len(cam_topics))]
        for ith, cam_topic in enumerate(cam_topics):
            for connection in reader.connections:
                if connection.topic == cam_topic:
                    print(f'Reading messages for {cam_topic}')
                    cams[ith].total_frames = len(list(reader.messages(connections=[connection])))
                    cams[ith].blur_regions = [[] for _ in range(cams[ith].total_frames)]
                    
                    # read images
                    images = []
                    for connection, timestamp, rawdata in reader.messages(connections=[connection]):
                        msg = self.typestore.deserialize_ros1(rawdata, connection.msgtype)
                        image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
                        images.append(image)
                    cams[ith].set_images(images)

                    cams[ith].msg_list = list(reader.messages(connections=[connection]))
        
        return cams