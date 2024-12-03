# path
from pathlib import Path
import sys

# other
from blur_face_manual.Application import Application

if __name__ == '__main__':
    
    if len(sys.argv) == 2:
        bag_file = Path(sys.argv[1])
        export_path = ""
    elif len(sys.argv) == 3:
        bag_file = Path(sys.argv[1])
        export_path = sys.argv[2]
    else:
        bag_file = Path('/home/jiahao/Downloads/1710755621-2024-03-18-10-02-36-1.bag')
        export_path = ""
        print("Usage: python blur_face_manual.py <path_to_bag_file>")
        print(f"Using default bag file {bag_file}")

    app = Application(bag_file, export_path)
    app.run()