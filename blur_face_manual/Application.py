# rosbags
from rosbags.typesys import get_typestore, Stores
from rosbags.highlevel import AnyReader, AnyReaderError
from rosbags.rosbag1 import Writer, WriterError
from rosbags.typesys.stores.ros1_noetic import sensor_msgs__msg__CompressedImage as CompressedImage

# opencv
import cv2
import numpy as np
from cv_bridge import CvBridge

# enum
from enum import Enum

# copy
import copy

# pathlib
from pathlib import Path

# other files
from blur_face_manual.BlurRegion import BlurRegion, draw_crosshair, blur_image
from blur_face_manual.Cam import Cam
from blur_face_manual.SaveFileHandler import SaveFileHandler


class Action(Enum):
    PASSTHROUGH = 1
    FILTER = 2

class DisplayType(Enum):
    PREBLUR = 1
    BLURRED = 2

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
        self.threashold_distance = 30
        
        self.render_type = DisplayType.PREBLUR

        # # process passthrough topics and other topics
        # self.process_passthrough_and_other_topics()

        # process camera topics
        self.create_window()
        self.register_callbacks()

        for ith in range(3):
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

    def check_if_move_enoughed_distance(self, start_x, start_y, end_x, end_y):
        moved_x = (end_x - start_x)**2
        moved_y = (end_y - start_y)**2
        both_moved = moved_x != 0 and moved_y != 0
        moved_enough = moved_x + moved_y > self.threashold_distance**2
        return both_moved and moved_enough

    
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

            self.cam[ith].moved_enoughed_distance = False

        elif event == cv2.EVENT_MOUSEMOVE:
            self.cam[ith].moved_enoughed_distance = self.check_if_move_enoughed_distance(self.cam[ith].drag_start_x, self.cam[ith].drag_start_y, x, y)            
            self.cam[ith].drag_end_x = x
            self.cam[ith].drag_end_y = y

            # position
            self.cam[ith].mouse_x, self.cam[ith].mouse_y = x, y

        elif event == cv2.EVENT_LBUTTONUP:
            self.cam[ith].dragging = False
            self.cam[ith].moved_enoughed_distance = self.check_if_move_enoughed_distance(self.cam[ith].drag_start_x, self.cam[ith].drag_start_y, x, y)
            self.cam[ith].drag_end_x = x
            self.cam[ith].drag_end_y = y

            if self.cam[ith].moved_enoughed_distance:
                # add blur region from dragged region
                blur_region = BlurRegion()
                blur_region.set_region(self.cam[ith].drag_start_x, self.cam[ith].drag_start_y, self.cam[ith].drag_end_x, self.cam[ith].drag_end_y)
                self.cam[ith].blur_regions[self.current_frame[ith]].append(blur_region)

                # save previous region
                self.cam[ith].last_region = copy.deepcopy(blur_region)
            else:
                # add blur region from saved width and height
                if self.cam[ith].last_region:
                    blur_region = copy.deepcopy(self.cam[ith].last_region)
                    blur_region.set_bottom_right_corner(x, y)
                    self.cam[ith].blur_regions[self.current_frame[ith]].append(blur_region)

        elif event == cv2.EVENT_MBUTTONDOWN or event == cv2.EVENT_RBUTTONDOWN:
            self.erase_region_under_cursor()

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

    def increase_frame(self, num):
        for ith in range(3):
            self.current_frame[ith] = min(len(self.cam[ith].msg_list) - 1, self.current_frame[ith] + num)
        self.render_windows()

    def decrease_frame(self, num):
        for ith in range(3):
            self.current_frame[ith] = max(0, self.current_frame[ith] - num)
        self.render_windows()
    
    def render_windows(self):
        for ith in range(3):
            self.render_window(ith)

    def render_window(self, ith):
        # get base image
        window_content = self.get_image_at_frame(ith, self.current_frame[ith])

        # draw blur regions border or blur regions
        if self.render_type == DisplayType.PREBLUR:
            for region in self.cam[ith].blur_regions[self.current_frame[ith]]:
                region.draw_border(window_content)
        elif self.render_type == DisplayType.BLURRED:
            all_regions = self.cam[ith].blur_regions[self.current_frame[ith]]
            blur_image(window_content, all_regions)

        # draw cursor
        mouse_location = (self.cam[ith].mouse_x, self.cam[ith].mouse_y)
        if self.cam[ith].mouse_in_window & (not self.cam[ith].dragging):
            if self.cam[ith].last_region:
                cursor_region = copy.deepcopy(self.cam[ith].last_region)
                cursor_region.set_bottom_right_corner(self.cam[ith].mouse_x, self.cam[ith].mouse_y)
                cursor_region.draw_border_with_crosshair(window_content)
            else :
                draw_crosshair(window_content, mouse_location)

        # draw live blur regions while dragging
        if self.cam[ith].dragging and self.cam[ith].moved_enoughed_distance:
            live_region = BlurRegion()
            live_region.set_region(self.cam[ith].drag_start_x, self.cam[ith].drag_start_y, self.cam[ith].drag_end_x, self.cam[ith].drag_end_y)
            live_region.draw_border_with_crosshair(window_content)
        
        # update window
        cv2.imshow('cam'+str(ith), window_content)

    def write_regions_to_file(self, path):
        save_file_handler = SaveFileHandler()
        save_file_handler.write_to_save_file(path, self.cam)

    def read_regions_from_file(self, path):
        save_file_handler = SaveFileHandler()
        loaded_cam = save_file_handler.read_from_save_file(path)
        if loaded_cam:    
            for ith in range(3):
                self.cam[ith].blur_regions = loaded_cam[ith].blur_regions
        
    def read_images_from_bag(self, reader, cam_topics):     
        self.cam = [Cam() for _ in range(len(cam_topics))]
        self.current_frame = [0 for _ in range(len(cam_topics))]
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

    def export_data_to_bag(self):
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

    def increase_region_size(self):
        for ith in range(3):
            if self.cam[ith].mouse_in_window and self.cam[ith].last_region:
                self.cam[ith].last_region.increase_size(0.05)    
                self.render_window(ith)
    
    def decrease_region_size(self):
        for ith in range(3):
            if self.cam[ith].mouse_in_window and self.cam[ith].last_region:
                self.cam[ith].last_region.decrease_size(0.05)    
                self.render_window(ith)

    def confirm_and_increase_frame(self):
        added_region = False
        for ith in range(3):
            if self.cam[ith].mouse_in_window and self.cam[ith].last_region:
                blur_region = copy.deepcopy(self.cam[ith].last_region)
                blur_region.set_bottom_right_corner(self.cam[ith].mouse_x, self.cam[ith].mouse_y)
                self.cam[ith].blur_regions[self.current_frame[ith]].append(blur_region)
                added_region = True
        if added_region:
            self.increase_frame(1)

    def erase_region_under_cursor(self):
        for ith in range(3):
            if self.cam[ith].mouse_in_window:
                x = self.cam[ith].mouse_x
                y = self.cam[ith].mouse_y

                # # erase under crosshair
                # if self.cam[ith].last_region:
                #     x -= self.cam[ith].last_region.width // 2
                #     y -= self.cam[ith].last_region.height // 2
                
                for region in reversed(self.cam[ith].blur_regions[self.current_frame[ith]]):
                    if region.contains(x, y):
                        self.cam[ith].blur_regions[self.current_frame[ith]].remove(region)
                        self.render_window(ith)
                        break
    
    def set_current_frame_as_ratio(self, ratio):
        for ith in range(3):
            self.current_frame[ith] = max(0, int(ratio * len(self.cam[ith].msg_list)) - 1)

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
                self.erase_region_under_cursor()
            elif key == ord('d'):
                self.increase_frame(1)
            elif key == ord('c'):
                self.increase_frame(10)
            elif key == ord('q'):
                break
            elif key == ord('e'):
                self.export_data_to_bag()
            elif key == ord('w'):
                self.export_data_to_bag()
                self.write_regions_to_file(self.save_name)
            elif key == ord('r'):
                self.read_regions_from_file(self.save_name)
                self.render_windows()
            elif key == ord('b'):
                if self.render_type == DisplayType.BLURRED:
                    self.render_type = DisplayType.PREBLUR
                else:
                    self.render_type = DisplayType.BLURRED
                self.render_windows()
            elif key == ord('f'):
                self.increase_region_size()
            elif key == ord('v'):
                self.decrease_region_size()
            elif key == ord('1'):
                self.set_current_frame_as_ratio(0.1)
                self.render_windows()
            elif key == ord('2'):
                self.set_current_frame_as_ratio(0.2)
                self.render_windows()
            elif key == ord('3'):
                self.set_current_frame_as_ratio(0.3)
                self.render_windows()
            elif key == ord('4'):
                self.set_current_frame_as_ratio(0.4)
                self.render_windows()
            elif key == ord('5'):
                self.set_current_frame_as_ratio(0.5)
                self.render_windows()
            elif key == ord('6'):
                self.set_current_frame_as_ratio(0.6)
                self.render_windows()
            elif key == ord('7'):
                self.set_current_frame_as_ratio(0.7)
                self.render_windows()
            elif key == ord('8'):
                self.set_current_frame_as_ratio(0.8)
                self.render_windows()
            elif key == ord('9'):
                self.set_current_frame_as_ratio(0.9)
                self.render_windows()
            elif key == ord('0'):
                self.set_current_frame_as_ratio(1.0)
                self.render_windows()
            elif key == ord('o'):
                self.set_current_frame_as_ratio(0.0)
                self.render_windows()
            else:
                pass

        cv2.destroyAllWindows()

