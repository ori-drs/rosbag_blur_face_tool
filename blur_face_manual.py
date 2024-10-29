# path
from pathlib import Path
# rosbags
from rosbags.highlevel import AnyReader
from rosbags.rosbag1 import Writer
from rosbags.typesys import get_typestore, Stores
from rosbags.typesys.stores.ros1_noetic import sensor_msgs__msg__CompressedImage as CompressedImage
# opencv
import cv2
import numpy as np
from cv_bridge import CvBridge
# enum
from enum import Enum


class Action(Enum):
    PASSTHROUGH = 1
    FILTER = 2

class BlurRegion:
    def __init__(self, start_x, start_y, end_x, end_y):
        self.start_x = start_x
        self.start_y = start_y
        self.end_x = end_x
        self.end_y = end_y

    def contains(self, x, y):
        return self.start_x <= x <= self.end_x and self.start_y <= y <= self.end_y
    
    def draw_border(self, image, color, thickness):
        cv2.rectangle(image, (self.start_x, self.start_y), (self.end_x, self.end_y), color, thickness)


class Application:

    def __init__(self):

        # helper objects
        self.typestore = get_typestore(Stores.ROS1_NOETIC)
        self.bridge = CvBridge()

        # input bag
        reader_bag_path = Path('/home/jiahao/Downloads/1710755621-2024-03-18-10-02-36-1.bag')
        self.setup_reader(reader_bag_path)

        # output bag
        writer_bag_path = Path('new.bag')
        # self.setup_writer(writer_bag_path)

        # camera topics
        self.camera_topics = [
            '/alphasense_driver_ros/cam0/debayered/image/compressed',
            '/alphasense_driver_ros/cam1/debayered/image/compressed',
            '/alphasense_driver_ros/cam2/debayered/image/compressed'
            ]

        self.passthrough_topics = [
            '/alphasense_driver_ros/imu',
            '/hesai/pandar'
        ]

        self.other_topics_action = Action.FILTER

        self.current_frame = 0
        self.cam0_mouse_x = -1
        self.cam0_mouse_y = -1
        self.cam1_mouse_x = -1
        self.cam1_mouse_y = -1
        self.cam2_mouse_x = -1
        self.cam2_mouse_y = -1
        self.cam0_image = None
        self.cam1_image = None
        self.cam2_image = None
        self.cam0_display_image = None
        self.cam1_display_image = None
        self.cam2_display_image = None

        self.cam0_msg_list = []
        self.cam1_msg_list = []
        self.cam2_msg_list = []

        self.cam0_blur_regions = []
        self.cam1_blur_regions = []
        self.cam2_blur_regions = []





        # # process passthrough topics and other topics
        # self.process_passthrough_and_other_topics()


        # process camera topics
        self.initialize_window()
        self.load_images_from_bag()
        self.initialize_blur_regions()
        self.read_images_at_current_frame()
        self.generate_display_image()
        self.update_window()


    def process_image(self, input_image):
        return cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)

    def image_to_compressed_msg(self, image, header):
        _, compressed_image = cv2.imencode('.jpg', image)
        return CompressedImage(
            header=header,
            format='jpg',
            data=np.frombuffer(compressed_image, dtype=np.uint8),
        )

    def setup_reader(self, bag_path):
        self.reader = AnyReader([bag_path], default_typestore=self.typestore)
        self.reader.open()

    def close_reader(self):
        self.reader.close()

    def setup_writer(self, bag_path):
        self.writer = Writer(bag_path)
        self.writer.open()

    def close_writer(self):
        self.writer.close()


    def load_images_from_bag(self):
        # get the three lists of messages
        for connection in self.reader.connections:
            if connection.topic == self.camera_topics[0]:
                self.cam0_msg_list = list(self.reader.messages(connections=[connection]))
            elif connection.topic == self.camera_topics[1]:
                self.cam1_msg_list = list(self.reader.messages(connections=[connection]))
                self.cam1_blur_regions = [[] for _ in range(len(self.cam1_msg_list))]
            elif connection.topic == self.camera_topics[2]:
                self.cam2_msg_list = list(self.reader.messages(connections=[connection]))
                self.cam2_blur_regions = [[] for _ in range(len(self.cam2_msg_list))]

    def initialize_blur_regions(self):
        self.cam0_blur_regions = [[] for _ in range(len(self.cam0_msg_list))]
        self.cam1_blur_regions = [[] for _ in range(len(self.cam1_msg_list))]
        self.cam2_blur_regions = [[] for _ in range(len(self.cam2_msg_list))]

    def process_passthrough_and_other_topics(self):
        # process passthrough topics and other topics
        for connection in self.reader.connections:

            if connection.topic in self.camera_topics:
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

    
    
    # Mouse event callback function to update mouse position
    def cam0_mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # dragging
            dragging = True
            self.cam0_drag_start_x = x
            self.cam0_drag_start_y = y

        elif event == cv2.EVENT_MOUSEMOVE:
            # dragging
            if dragging:
                self.cam0_drag_end_x = x
                self.cam0_drag_end_y = y

            # position
            self.cam0_mouse_x, self.cam0_mouse_y = x, y

        elif event == cv2.EVENT_LBUTTONUP:
            # dragging
            dragging = False
            self.cam0_drag_end_x = x
            self.cam0_drag_end_y = y

            # add blur region
            blur_region = BlurRegion(self.cam0_drag_start_x, self.cam0_drag_start_y, self.cam0_drag_end_x, self.cam0_drag_end_y)
            self.cam0_blur_regions[self.current_frame].append(blur_region)
    
    def cam1_mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # dragging
            dragging = True
            self.cam1_drag_start_x = x
            self.cam1_drag_start_y = y

        elif event == cv2.EVENT_MOUSEMOVE:
            # dragging
            if dragging:
                self.cam1_drag_end_x = x
                self.cam1_drag_end_y = y

            # position
            self.cam1_mouse_x, self.cam1_mouse_y = x, y

        elif event == cv2.EVENT_LBUTTONUP:
            # dragging
            dragging = False
            self.cam1_drag_end_x = x
            self.cam1_drag_end_y = y

            # add blur region
            blur_region = BlurRegion(self.cam1_drag_start_x, self.cam1_drag_start_y, self.cam1_drag_end_x, self.cam1_drag_end_y)
            self.cam1_blur_regions[self.current_frame].append(blur_region)
        
    def cam2_mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # dragging
            dragging = True
            self.cam2_drag_start_x = x
            self.cam2_drag_start_y = y

        elif event == cv2.EVENT_MOUSEMOVE:
            # dragging
            if dragging:
                self.cam2_drag_end_x = x
                self.cam2_drag_end_y = y

            # position
            self.cam2_mouse_x, self.cam2_mouse_y = x, y

        elif event == cv2.EVENT_LBUTTONUP:
            # dragging
            dragging = False
            self.cam2_drag_end_x = x
            self.cam2_drag_end_y = y

            # add blur region
            blur_region = BlurRegion(self.cam2_drag_start_x, self.cam2_drag_start_y, self.cam2_drag_end_x, self.cam2_drag_end_y)
            self.cam2_blur_regions[self.current_frame].append(blur_region)

    def initialize_window(self):
        # display windows
        cv2.namedWindow('cam0', cv2.WINDOW_NORMAL)
        cv2.namedWindow('cam1', cv2.WINDOW_NORMAL)
        cv2.namedWindow('cam2', cv2.WINDOW_NORMAL)

        # set callbacks
        cv2.setMouseCallback('cam0', self.cam0_mouse_callback)
        cv2.setMouseCallback('cam1', self.cam1_mouse_callback)
        cv2.setMouseCallback('cam2', self.cam2_mouse_callback)

        # resize windows
        cv2.resizeWindow('cam0', 640, 480)
        cv2.resizeWindow('cam1', 640, 480)
        cv2.resizeWindow('cam2', 640, 480)

        # move windows
        cv2.moveWindow('cam0', 640, 0)
        cv2.moveWindow('cam1', 0, 0)
        cv2.moveWindow('cam2', 1280, 0)
        

    def read_images_at_current_frame(self):
        # get first msg
        cam0_connection, cam0_timestamp, cam0_rawdata = self.cam0_msg_list[self.current_frame]
        cam1_connection, cam1_timestamp, cam1_rawdata = self.cam1_msg_list[self.current_frame]
        cam2_connection, cam2_timestamp, cam2_rawdata = self.cam2_msg_list[self.current_frame]

        cam0_msg = self.reader.deserialize(cam0_rawdata, cam0_connection.msgtype)
        cam1_msg = self.reader.deserialize(cam1_rawdata, cam1_connection.msgtype)
        cam2_msg = self.reader.deserialize(cam2_rawdata, cam2_connection.msgtype)

        self.cam0_image = self.bridge.compressed_imgmsg_to_cv2(cam0_msg, desired_encoding='passthrough')
        self.cam1_image = self.bridge.compressed_imgmsg_to_cv2(cam1_msg, desired_encoding='passthrough')
        self.cam2_image = self.bridge.compressed_imgmsg_to_cv2(cam2_msg, desired_encoding='passthrough')

    def update_window(self):
        cv2.imshow('cam0', self.cam0_display_image)
        cv2.imshow('cam1', self.cam1_display_image)
        cv2.imshow('cam2', self.cam2_display_image)

    def increase_frame(self):
        self.current_frame += 1

    def decrease_frame(self):
        self.current_frame = max(0, self.current_frame - 1)

    def reset_display_image(self):
        self.cam0_display_image = self.cam0_image.copy()
        self.cam1_display_image = self.cam1_image.copy()
        self.cam2_display_image = self.cam2_image.copy()

    def draw_crosshair(self):
        # Draw horizontal and vertical lines to create the crosshair
        line_length = 20
        color = (0, 0, 255)  # Red color for the crosshair
        thickness = 2
        
        # Overlay a crosshair at the mouse position
        if self.cam0_mouse_x != -1 and self.cam0_mouse_y != -1:
            cv2.line(self.cam0_display_image, (self.cam0_mouse_x - line_length, self.cam0_mouse_y), (self.cam0_mouse_x + line_length, self.cam0_mouse_y), color, thickness)
            cv2.line(self.cam0_display_image, (self.cam0_mouse_x, self.cam0_mouse_y - line_length), (self.cam0_mouse_x, self.cam0_mouse_y + line_length), color, thickness)
        
        if self.cam1_mouse_x != -1 and self.cam1_mouse_y != -1:
            cv2.line(self.cam1_display_image, (self.cam1_mouse_x - line_length, self.cam1_mouse_y), (self.cam1_mouse_x + line_length, self.cam1_mouse_y), color, thickness)
            cv2.line(self.cam1_display_image, (self.cam1_mouse_x, self.cam1_mouse_y - line_length), (self.cam1_mouse_x, self.cam1_mouse_y + line_length), color, thickness)

        if self.cam2_mouse_x != -1 and self.cam2_mouse_y != -1:
            cv2.line(self.cam2_display_image, (self.cam2_mouse_x - line_length, self.cam2_mouse_y), (self.cam2_mouse_x + line_length, self.cam2_mouse_y), color, thickness)
            cv2.line(self.cam2_display_image, (self.cam2_mouse_x, self.cam2_mouse_y - line_length), (self.cam2_mouse_x, self.cam2_mouse_y + line_length), color, thickness)

    def draw_blur_regions_border(self):
        for region in self.cam0_blur_regions[self.current_frame]:
            region.draw_border(self.cam0_display_image, (0, 0, 255), 2)
        for region in self.cam1_blur_regions[self.current_frame]:
            region.draw_border(self.cam1_display_image, (0, 0, 255), 2)
        for region in self.cam2_blur_regions[self.current_frame]:
            region.draw_border(self.cam2_display_image, (0, 0, 255), 2)

    def generate_display_image(self):
        self.reset_display_image()
        self.draw_crosshair()
        self.draw_blur_regions_border()

    # def write_images_to_bag(self):
    #     # modify the image to gray
    #     output_image = self.process_image(input_image)
        
    #     # compress image
    #     output_msg = self.image_to_compressed_msg(output_image, input_msg.header)

    #     # write to new bag
    #     new_connection = self.writer.add_connection(connection.topic, message_type, msgdef=connection.msgdef, typestore=self.typestore)
    #     new_rawdata = self.typestore.serialize_ros1(output_msg, message_type)
    #     new_timestamp = timestamp
    #     self.writer.write(new_connection, new_timestamp, new_rawdata)


    def run(self):
        while True:
            self.generate_display_image()
            self.update_window()

            key = cv2.waitKey(1)
            if key == ord('q'):
                break
            elif key == 81:  # Left arrow key
                self.decrease_frame()
                self.read_images_at_current_frame()
                self.generate_display_image()
                self.update_window()
            elif key == 83:  # Right arrow key
                self.increase_frame()
                self.read_images_at_current_frame()
                self.generate_display_image()
                self.update_window()
            else:
                continue

        cv2.destroyAllWindows()
        self.close_reader()
        # self.close_writer()


app = Application()
app.run()