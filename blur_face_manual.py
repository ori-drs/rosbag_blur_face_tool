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


class Cam:
    def __init__(self):
        # data and blur regions
        self.msg_list = []
        self.blur_regions = []

        # images
        self.image = None
        self.display_image = None
        
        # mouse position
        self.mouse_x = -1
        self.mouse_y = -1

        # dragging
        self.dragging = False
        self.drag_start_x = 0
        self.drag_start_y = 0
        self.drag_end_x = 0
        self.drag_end_y = 0
        self.moved_enoughed_distance = False

        # previous region
        self.have_previous_region = False
        self.previous_width = 0
        self.previous_height = 0

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
        self.threashold_distance = 30
        self.cam = [Cam() for _ in range(3)]

        # # process passthrough topics and other topics
        # self.process_passthrough_and_other_topics()

        # process camera topics
        self.create_window()
        self.register_callbacks()

        for ith in range(3):
            self.load_images_from_bag(ith)
            self.initialize_blur_regions(ith)
            self.read_images_at_current_frame(ith)
            self.generate_display_image(ith)
            self.update_window(ith)


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


    def load_images_from_bag(self, ith):
        # get the three lists of messages
        for connection in self.reader.connections:
            if connection.topic == self.camera_topics[ith]:
                self.cam[ith].msg_list = list(self.reader.messages(connections=[connection]))
                self.cam[ith].blur_regions = [[] for _ in range(len(self.cam[ith].msg_list))]

    def initialize_blur_regions(self, ith):
        self.cam[ith].blur_regions = [[] for _ in range(len(self.cam[ith].msg_list))]

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
    def mouse_callback(self, event, x, y, flags, ith):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.cam[ith].dragging = True

            self.cam[ith].drag_start_x = x
            self.cam[ith].drag_start_y = y
            self.cam[ith].drag_end_x = x
            self.cam[ith].drag_end_y = y

        elif event == cv2.EVENT_MOUSEMOVE:
            self.cam[ith].drag_end_x = x
            self.cam[ith].drag_end_y = y

            if self.cam[ith].dragging:
                self.cam[ith].moved_enoughed_distance = (self.cam[ith].drag_end_x - self.cam[ith].drag_start_x)**2 + (self.cam[ith].drag_end_y - self.cam[ith].drag_start_y)**2 > self.threashold_distance**2

            # position
            self.cam[ith].mouse_x, self.cam[ith].mouse_y = x, y

        elif event == cv2.EVENT_LBUTTONUP:
            self.cam[ith].drag_end_x = x
            self.cam[ith].drag_end_y = y
            self.cam[ith].moved_enoughed_distance = (self.cam[ith].drag_end_x - self.cam[ith].drag_start_x)**2 + (self.cam[ith].drag_end_y - self.cam[ith].drag_start_y)**2 > self.threashold_distance**2

            if self.cam[ith].moved_enoughed_distance:
                # add blur region from dragged region
                blur_region = BlurRegion(self.cam[ith].drag_start_x, self.cam[ith].drag_start_y, self.cam[ith].drag_end_x, self.cam[ith].drag_end_y)
                self.cam[ith].blur_regions[self.current_frame].append(blur_region)

                # save width and height
                self.cam[ith].previous_width = abs(self.cam[ith].drag_end_x - self.cam[ith].drag_start_x)
                self.cam[ith].previous_height = abs(self.cam[ith].drag_end_y - self.cam[ith].drag_start_y)
                self.cam[ith].have_previous_region = True
            else:
                # add blur region from saved width and height
                if self.cam[ith].have_previous_region:
                    blur_region = BlurRegion(x - self.cam[ith].previous_width, 
                                             y - self.cam[ith].previous_height, 
                                             x,
                                             y)
                    self.cam[ith].blur_regions[self.current_frame].append(blur_region)

            self.cam[ith].dragging = False
        
        self.generate_display_image(ith)
        self.update_window(ith)

    def create_window(self):
        # display windows
        cv2.namedWindow('cam0', cv2.WINDOW_NORMAL)
        cv2.namedWindow('cam1', cv2.WINDOW_NORMAL)
        cv2.namedWindow('cam2', cv2.WINDOW_NORMAL)

        # resize windows
        cv2.resizeWindow('cam0', 640, 480)
        cv2.resizeWindow('cam1', 640, 480)
        cv2.resizeWindow('cam2', 640, 480)

        # move windows
        cv2.moveWindow('cam0', 640, 0)
        cv2.moveWindow('cam1', 0, 0)
        cv2.moveWindow('cam2', 1280, 0)

    def register_callbacks(self):
        cv2.setMouseCallback('cam0', self.mouse_callback, param=0)
        cv2.setMouseCallback('cam1', self.mouse_callback, param=1)
        cv2.setMouseCallback('cam2', self.mouse_callback, param=2)

    def read_images_at_current_frame(self, ith):
        # get first msg
        camith_connection, camith_timestamp, camith_rawdata = self.cam[ith].msg_list[self.current_frame]
        camith_msg = self.reader.deserialize(camith_rawdata, camith_connection.msgtype)
        self.cam[ith].image = self.bridge.compressed_imgmsg_to_cv2(camith_msg, desired_encoding='passthrough')

    def update_window(self, ith):
        cv2.imshow('cam'+str(ith), self.cam[ith].display_image)

    def increase_frame(self):
        self.current_frame += 1

    def decrease_frame(self):
        self.current_frame = max(0, self.current_frame - 1)

    def reset_display_image(self, ith):
        self.cam[ith].display_image = self.cam[ith].image.copy()

    def draw_crosshair(self, ith):
        # Draw horizontal and vertical lines to create the crosshair
        line_length = 20
        color = (0, 0, 255)  # Red color for the crosshair
        thickness = 2
        
        # Overlay a crosshair at the mouse position
        if self.cam[ith].mouse_x != -1 and self.cam[ith].mouse_y != -1:
            cv2.line(self.cam[ith].display_image, (self.cam[ith].mouse_x - line_length, self.cam[ith].mouse_y), (self.cam[ith].mouse_x + line_length, self.cam[ith].mouse_y), color, thickness)
            cv2.line(self.cam[ith].display_image, (self.cam[ith].mouse_x, self.cam[ith].mouse_y - line_length), (self.cam[ith].mouse_x, self.cam[ith].mouse_y + line_length), color, thickness)

    def draw_crosshair_or_previous_region(self, ith):
        # Draw horizontal and vertical lines to create the crosshair
        line_length = 20
        color = (0, 0, 255)  # Red color for the crosshair
        thickness = 2

        if self.cam[ith].mouse_x != -1 and self.cam[ith].mouse_y != -1:
            if self.cam[ith].have_previous_region:
                cv2.rectangle(self.cam[ith].display_image, 
                              (self.cam[ith].mouse_x - self.cam[ith].previous_width, 
                               self.cam[ith].mouse_y - self.cam[ith].previous_height),
                              (self.cam[ith].mouse_x,
                               self.cam[ith].mouse_y),
                              color, thickness)
            else :
                cv2.line(self.cam[ith].display_image, (self.cam[ith].mouse_x - line_length, self.cam[ith].mouse_y), (self.cam[ith].mouse_x + line_length, self.cam[ith].mouse_y), color, thickness)
                cv2.line(self.cam[ith].display_image, (self.cam[ith].mouse_x, self.cam[ith].mouse_y - line_length), (self.cam[ith].mouse_x, self.cam[ith].mouse_y + line_length), color, thickness)
        
    def draw_blur_regions_border(self, ith):
        for region in self.cam[ith].blur_regions[self.current_frame]:
            region.draw_border(self.cam[ith].display_image, (0, 0, 255), 2)

    def draw_live_blur_regions(self, ith):
        if self.cam[ith].dragging and self.cam[ith].moved_enoughed_distance:
            cv2.rectangle(self.cam[ith].display_image, (self.cam[ith].drag_start_x, self.cam[ith].drag_start_y), (self.cam[ith].drag_end_x, self.cam[ith].drag_end_y), (0, 0, 255), 2)            

    def generate_display_image(self, ith):
        self.reset_display_image(ith)
        self.draw_crosshair_or_previous_region(ith)
        self.draw_blur_regions_border(ith)
        self.draw_live_blur_regions(ith)

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
            key = cv2.waitKey(1)
            if key == ord('q'):
                break
            elif key == 81:  # Left arrow key
                self.decrease_frame()
                for ith in range(3):
                    self.read_images_at_current_frame(ith)
                    self.generate_display_image(ith)
                    self.update_window(ith)
            elif key == 83:  # Right arrow key
                self.increase_frame()
                for ith in range(3):
                    self.read_images_at_current_frame(ith)
                    self.generate_display_image(ith)
                    self.update_window(ith)
            else:
                continue

        cv2.destroyAllWindows()
        self.close_reader()
        # self.close_writer()


app = Application()
app.run()