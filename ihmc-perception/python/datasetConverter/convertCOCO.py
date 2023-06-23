import json
import cv2
import os
import matplotlib.pyplot as plt
import shutil

"""
ConvertCOCO is a helper class that converts object detection datasets in COCO format to the YOLOv8 format.

COCO stores infomation about bounding boxes and semantic class in a json file
"""
class ConvertCOCO:
    """
    input_path - path to folder containing image dataset
    output_path - path to folder that renamed images should be saved in
    annotationFile - path to annotation json file in COCO format
    """
    def __init__(self, path):
        self.input_path = path + "raw/ms-coco/train2017" # path to images
        self.output_path = path + "processed/COCO/" # path to save renamed images
        annotFile = path + "raw/ms-coco/annotations/instances_train2017.json"

        try:
            os.mkdir(path + "processed/COCO")
            os.mkdir(path + "processed/COCO/labels")
            os.mkdir(path + "processed/COCO/images")
        except:
            print("VOC dataset has already been processed... exiting program")
            exit

        f = open(annotFile) # '/home/jcornette/.ihmc/datasets/raw/ms-coco/annotations/captions_train2017.json')
        self.data = json.load(f)
        f.close()

        self.file_names = []
        self.load_images_from_folder(self.input_path)


    def load_images_from_folder(self, folder):
        count = 0
        for filename in os.listdir(folder):
            source = os.path.join(folder,filename)
            destination = f"{self.output_path}images/img{count}.jpg"

            try:
                shutil.copy(source, destination)
                #print("File copied successfully.")
            # If source and destination are same
            except shutil.SameFileError:
                print(f"Source and destination represents the same file: {source} {destination}")

            self.file_names.append(filename)
            count += 1

    def get_img_ann(self, image_id):
        img_ann = []
        isFound = False
        for ann in self.data['annotations']:
            if ann['image_id'] == image_id:
                img_ann.append(ann)
                isFound = True
        if isFound:
            return img_ann
        else:
            return None
        
    def get_img(self, filename):
        for img in self.data['images']:
            if img['file_name'] == filename:
                return img
        
    def convert_coco(self):

        count = 0

        for filename in self.file_names:
            # Extracting image info from json
            img = self.get_img(filename)
            img_id = img['id']
            img_w = img['width']
            img_h = img['height']

            # Get Annotations for this image
            img_ann = self.get_img_ann(img_id)

            if img_ann:
                # Opening output annotation file for current image
                file_object = open(f"{self.output_path}labels/img{count}.txt", "a")

                for ann in img_ann:
                    current_category = ann['category_id'] - 1 # yolo format labels start from 0 
                    current_bbox = ann['bbox']
                    x = current_bbox[0]
                    y = current_bbox[1]
                    w = current_bbox[2]
                    h = current_bbox[3]
                    
                    # Finding midpoints
                    x_centre = (x + (x+w))/2
                    y_centre = (y + (y+h))/2
                    
                    # Normalization
                    x_centre = x_centre / img_w
                    y_centre = y_centre / img_h
                    w = w / img_w
                    h = h / img_h
                    
                    # Limiting upto fix number of decimal places
                    x_centre = format(x_centre, '.6f')
                    y_centre = format(y_centre, '.6f')
                    w = format(w, '.6f')
                    h = format(h, '.6f')
                        
                    # Writing current object 
                    file_object.write(f"{current_category} {x_centre} {y_centre} {w} {h}\n")

                file_object.close()
            count += 1  # This should be outside the if img_ann block.