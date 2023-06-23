#! /usr/bin/env python3

import os
import argparse
from util import *

path = '/home/jcornette/Desktop/dev/mergeDataset'

if __name__ == '__main__':

    parser = argparse.ArgumentParser()

    parser.add_argument("--path", help="path to search for files", type=str)
    parser.add_argument("--listRaw", help="List all raw datasets used", action="store_true")
    parser.add_argument("--info", help="file name for which to print info", action="store_true")
    parser.add_argument("--convertCOCO", help="convert downloaded COCO JSON format dataset to YOLO txt format", action="store_true")
    parser.add_argument("--convertVOC", help="convert downloaded VOC XML format dataset to YOLO txt format", action="store_true")
    parser.add_argument("--convertOID", help="convert downloaded OID XML format dataset to YOLO txt format", action="store_true")
    parser.add_argument("--validate", help="Validate that the conversion is accurate", type=str)
    parser.add_argument("--dataset", help="name of dataset you wish to validate", type=str)
    parser.add_argument("--view", help="file name of image to preview", type=str)

    parser.add_argument("--inputImage", help="path to images in raw dataset", type=str)
    parser.add_argument("--inputAnn", help="path to input annotation files", type=str)
    parser.add_argument("--output", help="path to output image/label files", type=str)

    args = parser.parse_args()
    home = os.path.expanduser('~')
    path = args.path if args.path else home + '/.ihmc/datasets/'

    if args.listRaw:
        files = sorted(os.listdir(path))
        print_contents(files)

    if args.view:
        view_image(path + args.view)

    if args.info:
        print_dataset_info()

    if args.convertCOCO:
        inputImage = args.inputImage if args.inputImage else home + '/.ihmc/datasets/raw/COCO/images/'
        inputAnn = args.inputAnn if args.inputAnn else home + '/.ihmc/datasets/raw/COCO/labels/'
        outputPath = args.output if args.output else home + '/.ihmc/datasets/processed/COCO/'
        convert_coco(inputImage, inputAnn, outputPath)

    if args.convertVOC:
        input = args.inputImage if args.inputImage else home + '/.ihmc/datasets/raw/VOC/images/'
        inputAnn = args.inputAnn if args.inputAnn else home + '/.ihmc/datasets/raw/VOC/labels/'
        outputPath = args.output if args.output else home + '/.ihmc/datasets/processed/VOC/'
        convert_voc(inputImage, inputAnn, outputPath)

    if args.convertOID:
        input = args.inputImage if args.input else home + '/.ihmc/datasets/raw/OID/images/'
        inputAnn = args.inputAnn if args.input else home + '/.ihmc/datasets/raw/OID/labels/'
        outputPath = args.output if args.output else home + '/.ihmc/datasets/processed/OID/'
        convert_OID(inputImage, inputAnn, outputPath)

    if args.validate:
        validate(path, args.validate, args.dataset)