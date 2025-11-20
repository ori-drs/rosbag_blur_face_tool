# path
from pathlib import Path
import sys

# other
from blur_face_manual.Application import Application

if __name__ == '__main__':
    
    # topics - frontier v7
    camera_topics = [ '/alphasense_driver_ros/cam0/debayered/image/compressed', 
                    '/alphasense_driver_ros/cam1/debayered/image/compressed',
                    '/alphasense_driver_ros/cam2/debayered/image/compressed']
    passthrough_topics = [ '/alphasense_driver_ros/imu',
                            '/hesai/pandar']

    bag_file = Path('/home/jiahao/Downloads/1710755621-2024-03-18-10-02-36-1.bag')
    export_folder = "./test1/"
    save_file_folder = "./test1/"

    app = Application(bag_file, save_file_folder, export_folder, camera_topics, passthrough_topics)
    app.export_to_bag()