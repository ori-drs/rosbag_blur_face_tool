# blur_face_manual
from blur_face_manual.BlurRegion import BlurRegion
from blur_face_manual.Cam import Cam

# path
from pathlib import Path

class SaveFileHandler:

    def __init__(self, path):
        self.path = path

    def write_to_save_file(self, cams):
        with open(self.path, 'w') as f:
            for ith, cam in enumerate(cams):
                f.write(f'cam{ith} {len(cam.blur_regions)}\n')
                f.write(str(cam))
        print(f'blurred regions written to "./{self.path}".')
    
    def read_from_save_file(self):
        if not Path(self.path).exists():
            print(f'file "./{self.path}" does not exist.')
            return None

        cams = []

        with open(self.path, 'r') as f:
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
        
        print(f'blurred regions read from "./{self.path}".')

        return cams
