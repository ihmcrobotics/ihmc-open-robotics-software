from PIL import Image
from convertCOCO import *
from convertVOC import *
from convertOID import *
from validator import *

def convert_coco(path):
    converter = ConvertCOCO(path)
    converter.convert_coco()
    print("The COCO dataset has been converted! Images have been added to the processed COCO folder. Check processed/COCO/labels for annotations")

def convert_voc(path):
    converter = ConvertVOC(path)
    converter.convert()
    converter.move_images_to_processed()
    print("The VOC dataset has been converted! Images have been added to the processed VOC folder. Check processed/VOC/labels for annotations")

def convert_OID(path):
    ConvertOID(path)

def validate(path, fileName, dataset):
    path = f"{path}processed/{dataset}/"
    v = Validator(path, fileName)

def print_contents(files):
    print("All raw datasets currently downloaded: ")
    for file in files:
        print(" " + file)

def view_image(image):
    img = Image.open(image)
    img.show()

def print_dataset_info():
    f = open('datasetInfo.txt', 'r')
    print(f.read())


