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

class DisplayType(Enum):
    PREBLUR = 1
    BLURRED = 2

class BlurRegion:
    def __init__(self):
        pass

    def set_region(self, start_x, start_y, end_x, end_y):
        self.start_x = start_x
        self.start_y = start_y
        self.end_x = end_x
        self.end_y = end_y

    def contains(self, x, y):
        return self.start_x <= x <= self.end_x and self.start_y <= y <= self.end_y
    
    def draw_border(self, image, color, thickness):
        cv2.rectangle(image, (self.start_x, self.start_y), (self.end_x, self.end_y), color, thickness)
    
    def blur_region(self, image):
        region = image[self.start_y:self.end_y, self.start_x:self.end_x]
        average_color = region.mean(axis=(0, 1), dtype=int)
        image[self.start_y:self.end_y, self.start_x:self.end_x] = average_color
    
    def __str__(self):
        return f'{self.start_x} {self.start_y} {self.end_x} {self.end_y}'

    def from_str(self, s):
        self.start_x, self.start_y, self.end_x, self.end_y = map(int, s.split())
        return self

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
    
    def __str__(self):
        string = ''
        for frame, regions in enumerate(self.blur_regions):
            for region in regions:
                string += f'{frame} {region}\n'
        return string
    
    def from_str(self, s):
        lines = s.split('\n')
        self.blur_regions = [[] for _ in range(len(lines))]
        for line in lines:
            if line:
                frame, region_str = line.split(' ', 1)
                blur_region = BlurRegion()
                blur_region.from_str(region_str)
                self.blur_regions[int(frame)].append(blur_region)
        return self

class SaveFileHandler:

    def __init__(self):
        pass

    def write_to_save_file(self, path, cams):
        with open(path, 'w') as f:
            for ith, cam in enumerate(cams):
                f.write(f'cam{ith} {len(cam.blur_regions)}\n')
                f.write(str(cam))
    
    def read_from_save_file(self, path):
        cams = []

        with open(path, 'r') as f:
            lines = f.readlines()

            current_cam = None
            for line in lines:
                if line.startswith('cam'):
                    length = int(line[5:])
                    current_cam = Cam()
                    current_cam.blur_regions = [[] for _ in range(length)]
                    cams.append(current_cam)
                else:
                    index, region_str = line.split(' ', 1)
                    blur_region = BlurRegion().from_str(region_str)
                    current_cam.blur_regions[int(index)].append(blur_region)
            
        return cams

class Application:

    def __init__(self):

        # helper objects
        self.typestore = get_typestore(Stores.ROS1_NOETIC)
        self.bridge = CvBridge()

        # output bag
        writer_bag_path = Path('new.bag')
        # self.setup_writer(writer_bag_path)

        self.passthrough_topics = [
            '/alphasense_driver_ros/imu',
            '/hesai/pandar'
        ]

        # read images from bag
        bag = Path('/home/jiahao/Downloads/1710755621-2024-03-18-10-02-36-1.bag')
        cam0_topic = '/alphasense_driver_ros/cam0/debayered/image/compressed'
        cam1_topic = '/alphasense_driver_ros/cam1/debayered/image/compressed'
        cam2_topic = '/alphasense_driver_ros/cam2/debayered/image/compressed'
        cam_topics = [cam0_topic, cam1_topic, cam2_topic]
        self.read_images_from_bag(bag, cam_topics)

        self.other_topics_action = Action.FILTER

        self.current_frame = 0
        self.threashold_distance = 30
        
        self.render_type = DisplayType.PREBLUR

        # # process passthrough topics and other topics
        # self.process_passthrough_and_other_topics()

        # process camera topics
        self.create_window()
        self.register_callbacks()

        for ith in range(3):
            self.initialize_blur_regions(ith)
            self.read_images_at_current_frame(ith)
            self.render_window(ith)


    def process_image(self, input_image):
        return cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)

    def image_to_compressed_msg(self, image, header):
        _, compressed_image = cv2.imencode('.jpg', image)
        return CompressedImage(
            header=header,
            format='jpg',
            data=np.frombuffer(compressed_image, dtype=np.uint8),
        )

    def setup_writer(self, bag_path):
        self.writer = Writer(bag_path)
        self.writer.open()

    def close_writer(self):
        self.writer.close()

    def initialize_blur_regions(self, ith):
        self.cam[ith].blur_regions = [[] for _ in range(len(self.cam[ith].msg_list))]

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
                blur_region = BlurRegion()
                blur_region.set_region(self.cam[ith].drag_start_x, self.cam[ith].drag_start_y, self.cam[ith].drag_end_x, self.cam[ith].drag_end_y)
                self.cam[ith].blur_regions[self.current_frame].append(blur_region)

                # save width and height
                self.cam[ith].previous_width = abs(self.cam[ith].drag_end_x - self.cam[ith].drag_start_x)
                self.cam[ith].previous_height = abs(self.cam[ith].drag_end_y - self.cam[ith].drag_start_y)
                self.cam[ith].have_previous_region = True
            else:
                # add blur region from saved width and height
                if self.cam[ith].have_previous_region:
                    blur_region = BlurRegion()
                    blur_region.set_region( x - self.cam[ith].previous_width,
                                            y - self.cam[ith].previous_height, 
                                            x,
                                            y)
                    self.cam[ith].blur_regions[self.current_frame].append(blur_region)

            self.cam[ith].dragging = False
        
        self.render_window(ith)

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

    def get_image_at_frame(self, ith, frame):
        camith_connection, camith_timestamp, camith_rawdata = self.cam[ith].msg_list[frame]
        camith_msg = self.typestore.deserialize_ros1(camith_rawdata, camith_connection.msgtype)
        return self.bridge.compressed_imgmsg_to_cv2(camith_msg, desired_encoding='passthrough')

    def read_images_at_current_frame(self, ith):
        self.cam[ith].image = self.get_image_at_frame(ith, self.current_frame)

    def increase_frame(self):
        self.current_frame += 1

    def decrease_frame(self):
        self.current_frame = max(0, self.current_frame - 1)

    def draw_crosshair(self, ith):
        # Draw horizontal and vertical lines to create the crosshair
        line_length = 20
        color = (0, 0, 255)  # Red color for the crosshair
        thickness = 2
        
        # Overlay a crosshair at the mouse position
        if self.cam[ith].mouse_x != -1 and self.cam[ith].mouse_y != -1:
            cv2.line(self.cam[ith].display_image, (self.cam[ith].mouse_x - line_length, self.cam[ith].mouse_y), (self.cam[ith].mouse_x + line_length, self.cam[ith].mouse_y), color, thickness)
            cv2.line(self.cam[ith].display_image, (self.cam[ith].mouse_x, self.cam[ith].mouse_y - line_length), (self.cam[ith].mouse_x, self.cam[ith].mouse_y + line_length), color, thickness)

    def draw_crosshair_or_previous_region_at_mouse_location(self, image, ith):
        # Draw horizontal and vertical lines to create the crosshair
        line_length = 20
        color = (0, 0, 255)  # Red color for the crosshair
        thickness = 2

        if self.cam[ith].mouse_x != -1 and self.cam[ith].mouse_y != -1:
            if self.cam[ith].have_previous_region:
                cv2.rectangle(image, 
                              (self.cam[ith].mouse_x - self.cam[ith].previous_width, 
                               self.cam[ith].mouse_y - self.cam[ith].previous_height),
                              (self.cam[ith].mouse_x,
                               self.cam[ith].mouse_y),
                              color, thickness)
            else :
                cv2.line(image, (self.cam[ith].mouse_x - line_length, self.cam[ith].mouse_y), (self.cam[ith].mouse_x + line_length, self.cam[ith].mouse_y), color, thickness)
                cv2.line(image, (self.cam[ith].mouse_x, self.cam[ith].mouse_y - line_length), (self.cam[ith].mouse_x, self.cam[ith].mouse_y + line_length), color, thickness)
        
    def draw_live_blur_regions(self, image, ith):
            cv2.rectangle(image, (self.cam[ith].drag_start_x, self.cam[ith].drag_start_y), (self.cam[ith].drag_end_x, self.cam[ith].drag_end_y), (0, 0, 255), 2)            

    def draw_blur_regions_border(self, image, region_list, ):
        for region in region_list:
            region.draw_border(image, (0, 0, 255), 2)

    def blur_regions(self, image, region_list):
        for region in region_list:
            region.blur_region(image)

    def render_window(self, ith):
        window_content = self.cam[ith].image.copy()

        if self.render_type == DisplayType.PREBLUR:
            self.draw_blur_regions_border(window_content, self.cam[ith].blur_regions[self.current_frame])            
        elif self.render_type == DisplayType.BLURRED:
            self.blur_regions(window_content, self.cam[ith].blur_regions[self.current_frame])

        self.draw_crosshair_or_previous_region_at_mouse_location(window_content, ith)

        if self.cam[ith].dragging and self.cam[ith].moved_enoughed_distance:
            self.draw_live_blur_regions(window_content, ith)
        
        # update window
        cv2.imshow('cam'+str(ith), window_content)

    def save_regions_to_file(self, path):
        save_file_handler = SaveFileHandler()
        save_file_handler.write_to_save_file(path, self.cam)

    def load_regions_from_file(self, path):
        save_file_handler = SaveFileHandler()
        loaded_cam = save_file_handler.read_from_save_file(path)
        for ith in range(3):
            self.cam[ith].blur_regions = loaded_cam[ith].blur_regions
        
    def read_images_from_bag(self, path, cam_topics):        
        reader = AnyReader([path], default_typestore=self.typestore)
        reader.open()

        self.cam = [Cam() for _ in range(len(cam_topics))]
        for ith, cam_topic in enumerate(cam_topics):
            for connection in reader.connections:
                if connection.topic == cam_topic:
                    self.cam[ith].msg_list = list(reader.messages(connections=[connection]))
                    self.cam[ith].blur_regions = [[] for _ in range(len(self.cam[ith].msg_list))]

        reader.close()

    # def write_images_to_bag(self, path):
        

    #     for ith, cam in enumerate(self.cam):
    #         for frame, (connection, timestamp, rawdata) in enumerate(cam.msg_list):
    #             image = self.bridge.compressed_imgmsg_to_cv2(self.reader.deserialize(rawdata, connection.msgtype), desired_encoding='passthrough')
    #             for region in cam.blur_regions[frame]:
    #                 start_x, start_y, end_x, end_y = region.start_x, region.start_y, region.end_x, region.end_y
    #                 image[start_y:end_y, start_x:end_x] = cv2.GaussianBlur(image[start_y:end_y, start_x:end_x], (25, 25), 0)
    #             compressed_image = self.bridge.cv2_to_compressed_imgmsg(image)
    #             writer.write(connection, timestamp, self.typestore.serialize_ros1(compressed_image, connection.msgtype))

        

    # def write_images_to_bag(self):
    #     writer = Writer(path)
    #     writer.open()

    #     # write for each cam
    #     for cam in self.cam:
    #         connection, _, _ = cam.msg_list[0]
    #         writer.add_connection(connection.topic, connection.msgtype, msgdef=connection.msgdef, typestore=self.typestore)  

    #         # write for each frame
    #         for frame, (connection, timestamp, rawdata) in enumerate(cam.msg_list):
    #             image = self.bridge.compressed_imgmsg_to_cv2(self.reader.deserialize(rawdata, connection.msgtype), desired_encoding='passthrough')
    #             for region in cam.blur_regions[frame]:
    #                 start_x, start_y, end_x, end_y = region.start_x, region.start_y, region.end_x, region.end_y
    #                 image[start_y:end_y, start_x:end_x] = cv2.GaussianBlur(image[start_y:end_y, start_x:end_x], (25, 25), 0)
    #             compressed_image = self.bridge.cv2_to_compressed_imgmsg(image)
    #             writer.write(connection, timestamp, self.typestore.serialize_ros1(compressed_image, connection.msgtype))

    #     # modify the image to gray
    #     output_image = self.process_image(input_image)
        
    #     # compress image
    #     output_msg = self.image_to_compressed_msg(output_image, input_msg.header)

    #     # write to new bag
    #     new_connection = self.writer.add_connection(connection.topic, message_type, msgdef=connection.msgdef, typestore=self.typestore)
    #     new_rawdata = self.typestore.serialize_ros1(output_msg, message_type)
    #     new_timestamp = timestamp
    #     self.writer.write(new_connection, new_timestamp, new_rawdata)

    #     writer.close()


    def run(self):
        while True:
            key = cv2.waitKey(1)
            if key == ord('q'):
                break
            elif key == 81:  # Left arrow key
                self.decrease_frame()
                for ith in range(3):
                    self.read_images_at_current_frame(ith)
                    self.render_window(ith)
            elif key == 83:  # Right arrow key
                self.increase_frame()
                for ith in range(3):
                    self.read_images_at_current_frame(ith)
                    self.render_window(ith)
            elif key == ord('s'):
                self.save_regions_to_file('save.txt')
            elif key == ord('l'):
                self.load_regions_from_file('save.txt')
                for ith in range(3):
                    self.render_window(ith)
            elif key == ord('b'):
                self.render_type = DisplayType.BLURRED
                for ith in range(3):
                    self.render_window(ith)
            elif key == ord('p'):
                self.render_type = DisplayType.PREBLUR
                for ith in range(3):
                    self.render_window(ith)
            elif key == ord('w'):
                path = Path('new.bag')
                self.write_images_to_bag(path)
            else:
                continue

        cv2.destroyAllWindows()
        # self.close_writer()


app = Application()
app.run()