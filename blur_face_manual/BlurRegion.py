# enum
from enum import Enum

# OpenCV
import cv2

# Numpy
import numpy as np

class BorderShape(Enum):
    RECTANGLE = 1
    ELLIPSE = 2
    BOTH = 3

def draw_crosshair(image, mouse_location):
    # Draw horizontal and vertical lines to create the crosshair
    line_length = 20
    color = (0, 0, 255)
    thickness = 2

    if mouse_location[0] != -1 and mouse_location[1] != -1:
        cv2.line(image, (mouse_location[0] - line_length, mouse_location[1]), (mouse_location[0] + line_length, mouse_location[1]), color, thickness)
        cv2.line(image, (mouse_location[0], mouse_location[1] - line_length), (mouse_location[0], mouse_location[1] + line_length), color, thickness)


class BlurRegion:
    def __init__(self):
        self.shape = BorderShape.ELLIPSE
        pass

    def set_region(self, start_x, start_y, end_x, end_y):
        self.start_x = start_x
        self.start_y = start_y
        self.end_x = end_x
        self.end_y = end_y

        self.width = abs(self.end_x - self.start_x)
        self.height = abs(self.end_y - self.start_y)

        self.original_width = self.width
        self.original_ratio = self.height / self.width
        
        self.magnification = 1

    def set_bottom_right_corner(self, x, y):
        self.end_x = x
        self.end_y = y

        self.start_x = self.end_x - self.width
        self.start_y = self.end_y - self.height

    def increase_size(self, step):
        self.magnification += step

        self.width = int(self.original_width * self.magnification)
        self.height = int(self.original_width * self.magnification * self.original_ratio)

        # update end points
        self.end_x = self.start_x + self.width
        self.end_y = self.start_y + self.height
    
    def decrease_size(self, step):
        self.magnification = max(step, self.magnification - step)

        self.width = int(self.original_width * self.magnification)
        self.height = int(self.original_width * self.magnification * self.original_ratio)

        # update end points
        self.end_x = self.start_x + self.width
        self.end_y = self.start_y + self.height

    def contains(self, x, y):
        return self.start_x <= x <= self.end_x and self.start_y <= y <= self.end_y
    
    def draw_rectangle(self, image, color = (0, 0, 255), thickness = 2):
        cv2.rectangle(image, (self.start_x, self.start_y), (self.end_x, self.end_y), color, thickness)
    
    def draw_ellipse(self, image, color = (0, 0, 255), thickness = 2):
        cv2.ellipse(image, ((self.start_x + self.end_x) // 2, (self.start_y + self.end_y) // 2), (self.width // 2, self.height // 2), 0, 0, 360, color, thickness = thickness)

    def draw_border(self, image, color = (0, 0, 255), thickness = 2):
        if self.shape == BorderShape.RECTANGLE:
            self.draw_rectangle(image, color, thickness)
        elif self.shape == BorderShape.ELLIPSE:
            self.draw_ellipse(image, color, thickness)
        elif self.shape == BorderShape.BOTH:
            self.draw_rectangle(image, color, thickness)
            self.draw_ellipse(image, color, thickness)

    def draw_border_with_crosshair(self, image, color = (0, 0, 255), thickness = 2):
        self.draw_border(image, color, thickness)
        crosshair_x = (self.start_x + self.end_x) // 2
        crosshair_y = (self.start_y + self.end_y) // 2
        draw_crosshair(image, (crosshair_x, crosshair_y))
    
    def blur_region(self, image, shape = BorderShape.ELLIPSE):
        if self.shape == BorderShape.RECTANGLE:
            region = image[self.start_y:self.end_y, self.start_x:self.end_x]
            average_color = region.mean(axis=(0, 1), dtype=int)
            image[self.start_y:self.end_y, self.start_x:self.end_x] = average_color
        elif self.shape == BorderShape.ELLIPSE or self.shape == BorderShape.BOTH:
            # gaussian elliptical blur
            blur_strength = 101 # odd number
            mask = np.zeros(image.shape[:2], dtype=np.uint8)
            cv2.ellipse(mask, ((self.start_x + self.end_x) // 2, (self.start_y + self.end_y) // 2), (self.width // 2, self.height // 2), 0, 0, 360, 255, thickness=-1)
            blurred_region = cv2.GaussianBlur(image, (blur_strength, blur_strength), 0)
            image[mask == 255] = blurred_region[mask == 255]

            # # average blur
            # mask = np.zeros(image.shape[:2], dtype=np.uint8)  # Create a single-channel mask
            # cv2.ellipse(mask, ((self.start_x + self.end_x) // 2, (self.start_y + self.end_y) // 2), (self.width // 2, self.height // 2), 0, 0, 360, 255, thickness=-1)
            # average_color = cv2.mean(image, mask=mask)[:3]
            # image[mask == 255] = average_color
    
    def __str__(self):
        return f'{self.start_x} {self.start_y} {self.end_x} {self.end_y}'

    def from_str(self, s):
        self.set_region(*map(int, s.split(' ')))
        return self


def blur_image(image, region_list):
    for region in region_list:
        region.blur_region(image)
        