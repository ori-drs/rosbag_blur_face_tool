# blue_face_manual
from blur_face_manual.BlurRegion import BlurRegion, blur_image

# cv2
from cv_bridge import CvBridge

class Cam:
    def __init__(self):
        # data and blur regions
        self.msg_list = []
        self.compressed_imgmsg_list = []

        self.bridge = CvBridge()

        self.blur_regions = []
        self.current_frame = 0
        self.total_frames = 0
        self.timestamp_list = []

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

    def get_current_image(self):
        return self.get_image(self.current_frame)

    def get_current_timestamp(self):
        return self.get_timestamp(self.current_frame)
    
    def get_timestamp(self, frame):
        return self.timestamp_list[frame]

    def get_image(self, frame):
        return self.bridge.compressed_imgmsg_to_cv2(self.compressed_imgmsg_list[frame], desired_encoding='passthrough')

    def get_image_with_blur(self, frame):
        # original image
        image = self.get_image(frame)

        # blurred
        blur_image(image, self.blur_regions[frame])

        # return
        return image
    
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
