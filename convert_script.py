# path
from pathlib import Path
import sys

# other
from blur_face_manual.Application import Application

if __name__ == '__main__':
    

    bag_file = Path('/home/jiahao/Downloads/1710755621-2024-03-18-10-02-36-1.bag')
    export_folder = "./test1/"
    save_file_folder = "./test1/"

    app = Application(bag_file, save_file_folder, export_folder)
    app.export_to_bag()