import pandas as pd
import os.path


class ConvertOID:

    def __init__(self, path) -> None:
        
        # TODO: Modify with the name of the folder containing the images
        self.IMAGE_DIR = f"{path}raw/OID/images"

        # Classes tor train (TODO: Modify the names with the desired labels)
        # WARNING: first letter should be UPPER case
        trainable_classes = ["Car","Bus","Truck"]

        annotation_files = ["train-annotations-bbox.csv","test-annotations-bbox.csv"]

        # Get the codes for the trainable classes
        class_descriptions = pd.read_csv('class-descriptions-boxable.csv', names={"class_code","class_name"}, header=None)
        self.trainable_codes = [code for code,name in class_descriptions.values if name in trainable_classes]
        # trainable_codes = [code for code,name in class_descriptions.values] # For ALL CLASSES

        for filename in annotation_files:

            # Read the train da
            filename = "train-annotations-bbox.csv"
            df = pd.read_csv(filename)

            # Keep only the data for our training labels
            # Comment this line for ALL CLASSES
            df = df.loc[df['LabelName'].isin(self.trainable_codes)]

            # Save the bounding box data to the files
            df.apply(lambda x: self.SaveBoundingBoxToFile(x['ImageID'],x['LabelName'],x['XMin'],x['XMax'],x['YMin'],x['YMax']), axis=1)
    


    def SaveBoundingBoxToFile(self, image_id,label,x_min,x_max,y_min,y_max):
        # Check that the image exist:
        if os.path.isfile(self.IMAGE_DIR + image_id + '.jpg'):

            # If the label file exist, append the new bounding box
            if os.path.isfile(self.IMAGE_DIR + image_id + '.txt'):
                with open(self.IMAGE_DIR + image_id+".txt",'a') as f:
                    f.write(' '.join([str(self.trainable_codes.index(label)),
                                    str(round((x_max+x_min)/2,6)),
                                    str(round((y_max+y_min)/2,6)),
                                    str(round(x_max-x_min,6)),
                                    str(round(y_max-y_min,6))])+'\n')
            else:
                with open(self.IMAGE_DIR+image_id+".txt",'w') as f:
                    f.write(' '.join([str(self.trainable_codes.index(label)),
                                    str(round((x_max+x_min)/2,6)),
                                    str(round((y_max+y_min)/2,6)),
                                    str(round(x_max-x_min,6)),
                                    str(round(y_max-y_min,6))])+'\n')