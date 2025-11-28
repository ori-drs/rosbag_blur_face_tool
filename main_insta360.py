# path
from pathlib import Path
import sys

# other
from blur_face_manual.Application import Application

if __name__ == '__main__':
    ####### topics - frontier v7
    # camera_topics = [ '/alphasense_driver_ros/cam0/debayered/image/compressed', 
    #                 '/alphasense_driver_ros/cam1/debayered/image/compressed',
    #                 '/alphasense_driver_ros/cam2/debayered/image/compressed']
    # passthrough_topics = [ '/alphasense_driver_ros/imu',
    #                         '/hesai/pandar']

    ####### topics - insta360
    camera_topics = [ '/cam0/image_raw', 
                      '/cam1/image_raw']
    passthrough_topics = [ '/imu/data_raw']
    
    if len(sys.argv) == 1:
        bag_file = Path('/home/jiahao/Downloads/1710755621-2024-03-18-10-02-36-1.bag')
        save_file_folder = ""
        export_folder = ""
        print("Usage: python blur_face_manual.py <path_to_bag_file>")
        print(f"Using default bag file {bag_file}")
    elif len(sys.argv) == 2:
        bag_file = Path(sys.argv[1])
        save_file_folder = ""
        export_folder = ""
    elif len(sys.argv) == 3:
        bag_file = Path(sys.argv[1])
        save_file_folder = sys.argv[2]
        export_folder = ""
    elif len(sys.argv) == 4:
        bag_file = Path(sys.argv[1])
        save_file_folder = sys.argv[2]
        export_folder = sys.argv[3]
    else:
        print("Usage: python blur_face_manual.py <path_to_bag_file> <save_path_prefix> <export_path>")
        sys.exit(1)
        

    app = Application(bag_file, save_file_folder, export_folder, camera_topics, passthrough_topics)
    app.run()