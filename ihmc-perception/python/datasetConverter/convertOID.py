import pandas as pd
import os.path

"""
ConvertOID is a helper class that converts Open Images Dataset bounding box csv format to the YOLOv8 txt format.

COCO stores infomation about bounding boxes and semantic class in a json file
"""
class ConvertOID:

    def __init__(self, inputImage, inputAnn, path) -> None:
        
        # TODO: Modify with the name of the folder containing the images
        self.IMAGE_DIR = inputImage
        self.inputImage = inputAnn
        self.inputAnn = inputImage
        self.outputAnn = f"{path}processed/OID/labels" # path + "processed/COCO/" # path to save renamed images
        self.outputImage = f"{path}processed/OID/images"

        try:
            os.mkdir(f"{path}/labels")
            os.mkdir(f"{path}/images")
        except:
            print("Path to output label/images already exists... exiting program")
            quit()

        # Classes tor train **NOT USED** (TODO: Modify the names with the desired labels)
        # WARNING: first letter should be UPPER case
        # trainable_classes = ["Car","Bus","Truck"]

        annotation_files = [f"{inputAnn}/train-annotations-bbox.csv", f"{inputAnn}/test-annotations-bbox.csv", f"{inputAnn}/validate-annotations-bbox.csv"]

        # Get the codes for the trainable classes
        class_descriptions = pd.read_csv(f"{inputAnn}/class-descriptions-boxable.csv", names=["LabelName","DisplayName"], header=0)
        # self.trainable_codes = [code for code,name in class_descriptions.values if name in trainable_classes]
        trainable_codes = [code for code,name in class_descriptions.values] # For ALL CLASSES

        for filename in annotation_files:
            if not os.path.isfile(filename):
                print(f"{filename} was not found, skipping it.")
                continue
            # Read the train da
            #filename = "train-annotations-bbox.csv"
            df = pd.read_csv(filename)

            # Keep only the data for our training labels
            # Comment this line for ALL CLASSES
            # df = df.loc[df['LabelName'].isin(self.trainable_codes)]

            # Save the bounding box data to the files
            df.apply(lambda x: self.SaveBoundingBoxToFile(x['ImageID'],x['LabelName'],x['XMin'],x['XMax'],x['YMin'],x['YMax']), axis=1)
    

    def SaveBoundingBoxToFile(self, image_id, label, x_min, x_max, y_min, y_max):
        # Check that the image exist:

        if os.path.isfile(self.IMAGE_DIR + image_id + '.jpg'):
            print(self.IMAGE_DIR + image_id + '.jpg')
            # If the label file exist, append the new bounding box
            if os.path.isfile(self.OUTPUT_DIR+ image_id + '.txt'):
                with open(self.OUTPUT_DIR + image_id+".txt",'a') as f:
                    f.write(' '.join([str(self.trainable_codes.index(label)),
                                    str(round((x_max+x_min)/2,6)),
                                    str(round((y_max+y_min)/2,6)),
                                    str(round(x_max-x_min,6)),
                                    str(round(y_max-y_min,6))])+'\n')
            else:
                with open(self.OUTPUT_DIR+image_id+".txt",'w') as f:
                    print("writing")
                    f.write(' '.join([str(self.trainable_codes.index(label)),
                                    str(round((x_max+x_min)/2,6)),
                                    str(round((y_max+y_min)/2,6)),
                                    str(round(x_max-x_min,6)),
                                    str(round(y_max-y_min,6))])+'\n')