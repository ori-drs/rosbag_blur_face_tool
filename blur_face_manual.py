# path
from pathlib import Path
# rosbags
from rosbags.highlevel import AnyReader, AnyReaderError
from rosbags.rosbag1 import Writer, WriterError
from rosbags.typesys import get_typestore, Stores
from rosbags.typesys.stores.ros1_noetic import sensor_msgs__msg__CompressedImage as CompressedImage
# opencv
import cv2
import numpy as np
from cv_bridge import CvBridge
# enum
from enum import Enum
import sys


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

        self.width = abs(self.end_x - self.start_x)
        self.height = abs(self.end_y - self.start_y)

    def contains(self, x, y):
        return self.start_x <= x <= self.end_x and self.start_y <= y <= self.end_y
    
    def draw_border(self, image, color, thickness):
        cv2.rectangle(image, (self.start_x, self.start_y), (self.end_x, self.end_y), color, thickness)
    
    def draw_border_offset(self, image, offset, color, thickness):
        cv2.rectangle(image, 
                    (offset[0] - self.width, 
                    offset[1] - self.height),
                    (offset[0],
                    offset[1]),
                    color, thickness)
    
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
        self.mouse_in_window = False

        # dragging
        self.dragging = False
        self.drag_start_x = 0
        self.drag_start_y = 0
        self.drag_end_x = 0
        self.drag_end_y = 0
        self.moved_enoughed_distance = False

        # previous region
        self.last_region = None
    
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
        print(f'blurred regions written to "./{path}".')
    
    def read_from_save_file(self, path):
        if not Path(path).exists():
            print(f'file "./{path}" does not exist.')
            return None

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
        
        print(f'blurred regions read from "./{path}".')

        return cams

# image processing
def draw_crosshair(image, mouse_location):
    # Draw horizontal and vertical lines to create the crosshair
    line_length = 20
    color = (0, 0, 255)
    thickness = 2

    if mouse_location[0] != -1 and mouse_location[1] != -1:
        cv2.line(image, (mouse_location[0] - line_length, mouse_location[1]), (mouse_location[0] + line_length, mouse_location[1]), color, thickness)
        cv2.line(image, (mouse_location[0], mouse_location[1] - line_length), (mouse_location[0], mouse_location[1] + line_length), color, thickness)

def blur_image(image, region_list):
    for region in region_list:
        region.blur_region(image)
        
class Application:

    def __init__(self, input_bag_path):

        # helper objects
        self.typestore = get_typestore(Stores.ROS1_NOETIC)
        self.bridge = CvBridge()


        cam0_topic = '/alphasense_driver_ros/cam0/debayered/image/compressed'
        cam1_topic = '/alphasense_driver_ros/cam1/debayered/image/compressed'
        cam2_topic = '/alphasense_driver_ros/cam2/debayered/image/compressed'
        cam_topics = [cam0_topic, cam1_topic, cam2_topic]
        self.passthrough_topics = [
            '/alphasense_driver_ros/imu',
            '/hesai/pandar'
        ]

        # process path
        self.input_bag_path = input_bag_path
        self.bag_name = self.input_bag_path.stem
        self.save_name = self.bag_name + '_save.txt'
        self.output_bag_name = self.bag_name + '_blurred.bag'

        # read image from bag
        reader = self.create_reader(self.input_bag_path)
        if reader:
            reader.open()
            self.read_images_from_bag(reader, cam_topics)
            reader.close()
        else:
            print('Quitting.')
            exit(1)
        

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
        # remove cursor from other windows
        for i in range(3):
            if self.cam[i].mouse_in_window:
                self.cam[i].mouse_in_window = False
                self.render_window(i)
        self.cam[ith].mouse_in_window = True
        
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

                # save previous region
                self.cam[ith].last_region = blur_region
            else:
                # add blur region from saved width and height
                if self.cam[ith].last_region:
                    blur_region = BlurRegion()
                    blur_region.set_region( x - self.cam[ith].last_region.width,
                                            y - self.cam[ith].last_region.height,
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

    def increase_frame(self, num):
        self.current_frame = min(len(self.cam[0].msg_list) - 1, self.current_frame + num)
        for ith in range(3):
            self.read_images_at_current_frame(ith)
            self.render_window(ith)

    def decrease_frame(self, num):
        self.current_frame = max(0, self.current_frame - num)
        for ith in range(3):
            self.read_images_at_current_frame(ith)
            self.render_window(ith)
    
    def render_window(self, ith):
        # get base image
        window_content = self.cam[ith].image.copy()

        # draw blur regions border or blur regions
        if self.render_type == DisplayType.PREBLUR:
            for region in self.cam[ith].blur_regions[self.current_frame]:
                region.draw_border(window_content, (0, 0, 255), 2)
        elif self.render_type == DisplayType.BLURRED:
            all_regions = self.cam[ith].blur_regions[self.current_frame]
            blur_image(window_content, all_regions)

        # draw cursor
        mouse_location = (self.cam[ith].mouse_x, self.cam[ith].mouse_y)
        if self.cam[ith].mouse_in_window & (not self.cam[ith].dragging):
            if self.cam[ith].last_region:
                self.cam[ith].last_region.draw_border_offset(window_content, mouse_location, (0, 0, 255), 2)
            else :
                draw_crosshair(window_content, mouse_location)

        # draw live blur regions while dragging
        if self.cam[ith].dragging and self.cam[ith].moved_enoughed_distance:
            live_region = BlurRegion()
            live_region.set_region(self.cam[ith].drag_start_x, self.cam[ith].drag_start_y, self.cam[ith].drag_end_x, self.cam[ith].drag_end_y)
            live_region.draw_border(window_content, (0, 0, 255), 2)
        
        # update window
        cv2.imshow('cam'+str(ith), window_content)

    def save_regions_to_file(self, path):
        save_file_handler = SaveFileHandler()
        save_file_handler.write_to_save_file(path, self.cam)

    def load_regions_from_file(self, path):
        save_file_handler = SaveFileHandler()
        loaded_cam = save_file_handler.read_from_save_file(path)
        if loaded_cam:    
            for ith in range(3):
                self.cam[ith].blur_regions = loaded_cam[ith].blur_regions
        
    def read_images_from_bag(self, reader, cam_topics):     
        self.cam = [Cam() for _ in range(len(cam_topics))]
        for ith, cam_topic in enumerate(cam_topics):
            for connection in reader.connections:
                if connection.topic == cam_topic:
                    print(f'Reading messages for {cam_topic}')
                    self.cam[ith].msg_list = list(reader.messages(connections=[connection]))
                    self.cam[ith].blur_regions = [[] for _ in range(len(self.cam[ith].msg_list))]

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

    def write_images_to_bag(self, writer):
        # for each camera
        for ith, cam in enumerate(self.cam):
            input_connection, _, _ = cam.msg_list[0]
            output_connection = writer.add_connection(input_connection.topic, input_connection.msgtype, msgdef=input_connection.msgdef, typestore=self.typestore)  

            # write for each frame
            for frame in range(len(cam.msg_list)):
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

    def write_data_to_bag(self):
        new_path = Path(self.output_bag_name)
                
        writer = self.create_writer(new_path)
        reader = self.create_reader(self.input_bag_path)
        if writer and reader:
            writer.open()
            reader.open()

            self.write_images_to_bag(writer)
            self.write_other_topics_to_bag(reader, writer)
            
            writer.close()
            reader.close()
            print(f'Bag file written to {writer.path}')
        else:
            return

    def confirm_and_increase_frame(self):
        added_region = False
        for ith in range(3):
            if self.cam[ith].mouse_in_window and self.cam[ith].last_region:
                blur_region = BlurRegion()
                blur_region.set_region(self.cam[ith].mouse_x - self.cam[ith].last_region.width,
                                        self.cam[ith].mouse_y - self.cam[ith].last_region.height,
                                        self.cam[ith].mouse_x,
                                        self.cam[ith].mouse_y)
                self.cam[ith].blur_regions[self.current_frame].append(blur_region)
                added_region = True
        if added_region:
            self.increase_frame(1)

    def run(self):
        while True:
            key = cv2.waitKey(1)
            if key == ord('z'):
                self.decrease_frame(10)
            elif key == ord('a'):
                self.decrease_frame(1)
            elif key == ord('s'):
                self.confirm_and_increase_frame()
            elif key == ord('x'):
                self.delete_region_under_cursor()
            elif key == ord('d'):
                self.increase_frame(1)
            elif key == ord('c'):
                self.increase_frame(10)
            elif key == ord('q'):
                break
            elif key == ord('w'):
                self.write_data_to_bag()
            elif key == ord('e'):
                self.save_regions_to_file(self.save_name)
            elif key == ord('r'):
                self.load_regions_from_file(self.save_name)
                for ith in range(3):
                    self.render_window(ith)
            elif key == ord('f'):
                if self.render_type == DisplayType.BLURRED:
                    self.render_type = DisplayType.PREBLUR
                else:
                    self.render_type = DisplayType.BLURRED
                for ith in range(3):
                    self.render_window(ith)
            else:
                pass

        cv2.destroyAllWindows()


if __name__ == '__main__':
    
    if len(sys.argv) == 2:
        bag_file = Path(sys.argv[1])
    else:
        bag_file = Path('/home/jiahao/Downloads/1710755621-2024-03-18-10-02-36-1.bag')
        print("Usage: python blur_face_manual.py <path_to_bag_file>")
        print(f"Using default bag file {bag_file}")

    app = Application(bag_file)
    app.run()