import h5py
import cv2
import numpy as np
import os
from hdf5_reader import *
from hdf5_converter import *
import argparse

if __name__ == '__main__':

    # 20221212_184748_PerceptionLog.hdf5
    # 20221212_184906_PerceptionLog.hdf5
    # 20221212_184940_PerceptionLog.hdf5

    # 20221216_141954_PerceptionLog.hdf5
    # 20221216_143619_PerceptionLog.hdf5
    # 20221216_144027_PerceptionLog.hdf5

    home = os.path.expanduser('~')

    path = home + '/.ihmc/logs/perception/'


    old_file = '20230207_214209_PerceptionLog.hdf5'
    new_file = old_file.replace('.hdf5', 'Fixed.hdf5')


    data = h5py.File(path + old_file, 'r')

    channels = collect_channels(data)

    conversions = []

    for channel in channels:
        conversions.append(dict(dtype=channel['dtype'], src_name=channel['name'], dst_name=channel['name'], count=channel['count']))


    # conversions = [dict(type='byte', src_name='ouster/depth/', dst_name='ouster/depth/', count=507),
    #                dict(type='float', src_name='ouster/sensor/position/', dst_name='ouster/sensor/position/', count=50),
    #                dict(type='float', src_name='ouster/sensor/orientation/', dst_name='ouster/sensor/orientation/', count=50)]

    convert_hdf5_file(path, old_file, new_file, conversions)




