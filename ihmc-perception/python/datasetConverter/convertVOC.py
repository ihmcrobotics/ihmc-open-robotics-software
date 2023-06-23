import xml.etree.ElementTree as ET
import glob
import os
import json

class ConvertVOC:

    def __init__(self, path) -> None:
        self.path = path
        self.classes = []
        self.input_dir = path + "raw/VOC2007/Annotations/"
        self.output_dir = path + "/processed/VOC/labels/"
        self.image_dir = path + "raw/VOC2007/JPEGImages/"
        try:
            os.mkdir(path + "processed/VOC")
            os.mkdir(path + "processed/VOC/labels")
            os.mkdir(path + "processed/VOC/images")
        except:
            print("VOC dataset has already been processed.")

    def xml_to_yolo_bbox(self, bbox, w, h):
        # xmin, ymin, xmax, ymax
        x_center = ((bbox[2] + bbox[0]) / 2) / w
        y_center = ((bbox[3] + bbox[1]) / 2) / h
        width = (bbox[2] - bbox[0]) / w
        height = (bbox[3] - bbox[1]) / h
        return [x_center, y_center, width, height]

    def yolo_to_xml_bbox(self, bbox, w, h):
        # x_center, y_center width heigth
        w_half_len = (bbox[2] * w) / 2
        h_half_len = (bbox[3] * h) / 2
        xmin = int((bbox[0] * w) - w_half_len)
        ymin = int((bbox[1] * h) - h_half_len)
        xmax = int((bbox[0] * w) + w_half_len)
        ymax = int((bbox[1] * h) + h_half_len)
        return [xmin, ymin, xmax, ymax]
    
    def move_images_to_processed(self):
        # identify all the jpg files in the images folder
        files = glob.glob(os.path.join(self.image_dir, '*.jpg'))
        for file in files:
            fileName = os.path.basename(file)
            os.rename(file, f"{self.path}processed/VOC/images/{fileName}" )

    def convert(self):

        # create the labels folder (output directory)
        if not os.path.isdir(self.output_dir):
            os.mkdir(self.output_dir)

        # identify all the xml files in the annotations folder (input directory)
        files = glob.glob(os.path.join(self.input_dir, '*.xml'))

        # loop through each 
        for fil in files:
            basename = os.path.basename(fil)
            filename = os.path.splitext(basename)[0]
            # check if the label contains the corresponding image file
            if not os.path.exists(os.path.join(self.image_dir, f"{filename}.jpg")):
                print(f"{filename} image does not exist!")
                continue

            result = []

            # parse the content of the xml file
            tree = ET.parse(fil)
            root = tree.getroot()
            width = int(root.find("size").find("width").text)
            height = int(root.find("size").find("height").text)

            for obj in root.findall('object'):
                label = obj.find("name").text
                # check for new classes and append to list
                if label not in self.classes:
                    self.classes.append(label)
                index = self.classes.index(label)
                pil_bbox = [int(x.text) for x in obj.find("bndbox")]
                yolo_bbox = self.xml_to_yolo_bbox(pil_bbox, width, height)
                # convert data to string
                bbox_string = " ".join([str(x) for x in yolo_bbox])
                result.append(f"{index} {bbox_string}")

            if result:
                # generate a YOLO format text file for each xml file
                with open(os.path.join(self.output_dir, f"{filename}.txt"), "w", encoding="utf-8") as f:
                    f.write("\n".join(result))

        # generate the classes file as reference
        with open('classes.txt', 'w', encoding='utf8') as f:
            f.write(json.dumps(self.classes))

        self.move_images_to_processed()