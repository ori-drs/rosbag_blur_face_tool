# opencv
import cv2

# enum
from enum import Enum

# copy
import copy

# path
from pathlib import Path

# blur_face_manual
from blur_face_manual.BlurRegion import BlurRegion, draw_crosshair, blur_image
from blur_face_manual.SaveFileHandler import SaveFileHandler
from blur_face_manual.BagFileHandler import BagFileHandler

class DisplayType(Enum):
    PREBLUR = 1
    BLURRED = 2

class Application:

    def __init__(self, input_bag_path, save_file_folder = None, export_folder = None):
        # convert to path
        input_bag_path = Path(input_bag_path)

        # helper objects
        self.BagFileHandler = BagFileHandler(input_bag_path, export_folder)
        self.SaveFileHandler = SaveFileHandler(save_file_folder + input_bag_path.stem + '_save.txt')

        # get cams
        self.cams = self.BagFileHandler.get_cams()

        # try to read regions from file
        self.read_regions_from_file()

        self.threashold_distance = 30
        
        self.render_type = DisplayType.PREBLUR

        # # process passthrough topics and other topics
        # self.process_passthrough_and_other_topics()

    def process_image(self, input_image):
        return cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)

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
            if self.cams[i].mouse_in_window:
                self.cams[i].mouse_in_window = False
                self.render_window(i)
        self.cams[ith].mouse_in_window = True
        
        if event == cv2.EVENT_LBUTTONDOWN:
            self.cams[ith].dragging = True
            self.cams[ith].drag_start_x = x
            self.cams[ith].drag_start_y = y
            self.cams[ith].drag_end_x = x
            self.cams[ith].drag_end_y = y

            self.cams[ith].moved_enoughed_distance = False

        elif event == cv2.EVENT_MOUSEMOVE:
            self.cams[ith].moved_enoughed_distance = self.check_if_move_enoughed_distance(self.cams[ith].drag_start_x, self.cams[ith].drag_start_y, x, y)            
            self.cams[ith].drag_end_x = x
            self.cams[ith].drag_end_y = y

            # position
            self.cams[ith].mouse_x, self.cams[ith].mouse_y = x, y

        elif event == cv2.EVENT_LBUTTONUP:
            self.cams[ith].dragging = False
            self.cams[ith].moved_enoughed_distance = self.check_if_move_enoughed_distance(self.cams[ith].drag_start_x, self.cams[ith].drag_start_y, x, y)
            self.cams[ith].drag_end_x = x
            self.cams[ith].drag_end_y = y

            if self.cams[ith].moved_enoughed_distance:
                # add blur region from dragged region
                blur_region = BlurRegion()
                blur_region.set_region(self.cams[ith].drag_start_x, self.cams[ith].drag_start_y, self.cams[ith].drag_end_x, self.cams[ith].drag_end_y)
                self.cams[ith].blur_regions[self.cams[ith].current_frame].append(blur_region)

                # save previous region
                self.cams[ith].last_region = copy.deepcopy(blur_region)
            else:
                # add blur region from saved width and height
                if self.cams[ith].last_region:
                    blur_region = copy.deepcopy(self.cams[ith].last_region)
                    blur_region.set_bottom_right_corner(x, y)
                    self.cams[ith].blur_regions[self.cams[ith].current_frame].append(blur_region)

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

    def increase_frame(self, num):
        for ith in range(3):
            self.cams[ith].current_frame = min(self.cams[ith].total_frames - 1, self.cams[ith].current_frame + num)
        self.render_windows()

    def decrease_frame(self, num):
        for ith in range(3):
            self.cams[ith].current_frame = max(0, self.cams[ith].current_frame - num)
        self.render_windows()
    
    def render_windows(self):
        for ith in range(3):
            self.render_window(ith)

    def render_window(self, ith):
        # get base image
        window_content = self.cams[ith].get_current_image()

        # print timestamp
        print(f"cam{ith} {self.cams[ith].get_current_timestamp()}")

        # draw blur regions border or blur regions
        if self.render_type == DisplayType.PREBLUR:
            for region in self.cams[ith].blur_regions[self.cams[ith].current_frame]:
                region.draw_border(window_content)
        elif self.render_type == DisplayType.BLURRED:
            all_regions = self.cams[ith].blur_regions[self.cams[ith].current_frame]
            blur_image(window_content, all_regions)

        # draw cursor
        mouse_location = (self.cams[ith].mouse_x, self.cams[ith].mouse_y)
        if self.cams[ith].mouse_in_window & (not self.cams[ith].dragging):
            if self.cams[ith].last_region:
                cursor_region = copy.deepcopy(self.cams[ith].last_region)
                cursor_region.set_bottom_right_corner(self.cams[ith].mouse_x, self.cams[ith].mouse_y)
                cursor_region.draw_border_with_crosshair(window_content)
            else :
                draw_crosshair(window_content, mouse_location)

        # draw live blur regions while dragging
        if self.cams[ith].dragging and self.cams[ith].moved_enoughed_distance:
            live_region = BlurRegion()
            live_region.set_region(self.cams[ith].drag_start_x, self.cams[ith].drag_start_y, self.cams[ith].drag_end_x, self.cams[ith].drag_end_y)
            live_region.draw_border_with_crosshair(window_content)
        
        # update window
        cv2.imshow('cam'+str(ith), window_content)        

    def read_regions_from_file(self):
        loaded_cam = self.SaveFileHandler.read_from_save_file()
        if loaded_cam:    
            for ith in range(3):
                self.cams[ith].blur_regions = loaded_cam[ith].blur_regions
        
    def increase_region_size(self):
        for ith in range(3):
            if self.cams[ith].mouse_in_window and self.cams[ith].last_region:
                self.cams[ith].last_region.increase_size(0.05)    
                self.render_window(ith)
    
    def decrease_region_size(self):
        for ith in range(3):
            if self.cams[ith].mouse_in_window and self.cams[ith].last_region:
                self.cams[ith].last_region.decrease_size(0.05)    
                self.render_window(ith)

    def confirm_and_increase_frame(self):
        added_region = False
        for ith in range(3):
            if self.cams[ith].mouse_in_window and self.cams[ith].last_region:
                blur_region = copy.deepcopy(self.cams[ith].last_region)
                blur_region.set_bottom_right_corner(self.cams[ith].mouse_x, self.cams[ith].mouse_y)
                self.cams[ith].blur_regions[self.cams[ith].current_frame].append(blur_region)
                added_region = True
        if added_region:
            self.increase_frame(1)

    def erase_region_under_cursor(self):
        for ith in range(3):
            if self.cams[ith].mouse_in_window:
                x = self.cams[ith].mouse_x
                y = self.cams[ith].mouse_y

                # # erase under crosshair
                # if self.RosbagHandler.cam[ith].last_region:
                #     x -= self.RosbagHandler.cam[ith].last_region.width // 2
                #     y -= self.RosbagHandler.cam[ith].last_region.height // 2
                
                for region in reversed(self.cams[ith].blur_regions[self.cams[ith].current_frame]):
                    if region.contains(x, y):
                        self.cams[ith].blur_regions[self.cams[ith].current_frame].remove(region)
                        self.render_window(ith)
                        break
    
    def set_current_frame_as_ratio(self, ratio):
        for ith in range(3):
            self.cams[ith].current_frame = max(0, int(ratio * self.cams[ith].total_frames) - 1)

    def export_to_bag(self):
        self.BagFileHandler.export_cams(self.cams)

    def run(self):
        # create windows
        self.create_window()
        self.register_callbacks()

        # render windows
        for ith in range(3):
            self.render_window(ith)

        # listen to key press
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
                # write save then export
                self.SaveFileHandler.write_to_save_file(self.cams)
                self.export_to_bag()
            elif key == ord('w'):
                # write save
                self.SaveFileHandler.write_to_save_file(self.cams)
            elif key == ord('r'):
                self.read_regions_from_file()
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
        
        # close windows
        cv2.destroyAllWindows()

