# numpy
import numpy as np

# OpenCV
import cv2

# rosbags
from rosbags.typesys import get_typestore, Stores
from rosbags.highlevel import AnyReader, AnyReaderError
from rosbags.rosbag1 import Writer, WriterError
from rosbags.typesys.stores.ros1_noetic import sensor_msgs__msg__CompressedImage as CompressedImage

# blur_face_manual
from blur_face_manual.Cam import Cam

class BagFileHandler:
    def __init__(self, path: str, export_folder: str):
        # input and output bag path
        self.input_bag_path = path
        self.output_bag_name = export_folder + self.input_bag_path.stem + '_blurred.bag'

        # cam topics
        self.cam_topics = [ '/alphasense_driver_ros/cam0/debayered/image/compressed', 
                            '/alphasense_driver_ros/cam1/debayered/image/compressed',
                            '/alphasense_driver_ros/cam2/debayered/image/compressed']

        # passthrough topics
        self.passthrough_topics = [ '/alphasense_driver_ros/imu',
                                    '/hesai/pandar']

    def create_reader(self, path):
        typestore = get_typestore(Stores.ROS1_NOETIC)
        try:
            reader = AnyReader([path], default_typestore = typestore)
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
        
    # read bag and output cam object
    def get_cams(self):
        # typestore to deserialize messages
        typestore = get_typestore(Stores.ROS1_NOETIC)
        
        # reader to read bag
        reader = self.create_reader(self.input_bag_path)

        # error check
        if reader is None:
            exit()

        # open
        reader.open()

        # initialize cam
        cams = [Cam() for _ in range(len(self.cam_topics))]

        # for each image connection, store compressed image messages
        for connection, timestamp, rawdata in reader.messages():
            # skip if not cam topics
            if connection.topic not in self.cam_topics:
                continue

            # log
            print(f'Reading message of timestamp {timestamp} for topic {connection.topic}')

            # get ith
            ith = self.cam_topics.index(connection.topic)

            # deserialize message
            msg = typestore.deserialize_ros1(rawdata, connection.msgtype)

            # store data
            cams[ith].compressed_imgmsg_list.append(msg)
            cams[ith].timestamp_list.append(timestamp)
            cams[ith].total_frames += 1
            cams[ith].blur_regions.append([])

        # close reader
        reader.close()
        
        # return
        return cams

    # write both cam and other topics to bag
    def export_cams(self, cams):
        # reader and writer
        reader = self.create_reader(self.input_bag_path)
        writer = self.create_writer(self.output_bag_name)

        # error check
        if reader is None or writer is None:
            return

        # open
        reader.open()
        writer.open()
        
        # typestore
        typestore = get_typestore(Stores.ROS1_NOETIC)

        # create connections
        reader_connections = []
        output_connections = []
        for connection in reader.connections:
            # skip if not in recognized topics
            if connection.topic not in self.passthrough_topics and connection.topic not in self.cam_topics:
                continue
            
            # add connection
            output_connection = writer.add_connection(connection.topic, connection.msgtype, msgdef=connection.msgdef, typestore=typestore)

            # store connections
            reader_connections.append(connection) # this is stored to provide indexing later
            output_connections.append(output_connection)

            # log
            print(f'Added connection {connection.topic}')

        # for each message
        for connection, timestamp, rawdata in reader.messages():
            # skip if not in recognized topics
            if connection.topic not in self.passthrough_topics and connection.topic not in self.cam_topics:
                continue
            
            # log
            print(f'Writing message of timestamp {timestamp} for topic {connection.topic}')
            
            # find output connection
            output_connection = output_connections[reader_connections.index(connection)]

            # check if connection is in cam topics
            if connection.topic in self.cam_topics:

                # check if blur regions are added
                ith = self.cam_topics.index(connection.topic)
                frame = cams[ith].timestamp_list.index(timestamp)
                if cams[ith].blur_regions[frame]:
                    # create new rawdata
                    new_image = cams[ith].get_image_with_blur(frame)
                    new_msg = self.image_to_compressed_msg(new_image, cams[ith].compressed_imgmsg_list[frame].header)
                    new_rawdata = typestore.serialize_ros1(new_msg, connection.msgtype)

                    # use the new rawdata
                    writer.write(output_connection, timestamp, new_rawdata)
                else:
                    # use the same rawdata
                    writer.write(output_connection, timestamp, rawdata)
            else:
                # write other topics using the same rawdata
                writer.write(output_connection, timestamp, rawdata)

        # close
        reader.close()
        writer.close()

        # log
        print(f'Bag file written to {writer.path}')

    def image_to_compressed_msg(self, image, header):
        _, compressed_image = cv2.imencode('.jpg', image)
        return CompressedImage(
            header=header,
            format='jpg',
            data=np.frombuffer(compressed_image, dtype=np.uint8),
        )