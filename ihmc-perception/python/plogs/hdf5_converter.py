import h5py
import numpy as np
from hdf5_reader import *

def convert_hdf5_file(path, src_file, dst_file, conversions):

    src_h5 = h5py.File(path + src_file, 'r')
    dst_h5 = h5py.File(path + dst_file, 'w')

    print_file_info(src_h5, src_file)

    for conversion in conversions:

        print("Conversion: ", conversion)

        for i in range(conversion['count']):
            if conversion['dtype'] == 'byte':
                copy_byte_dataset(src_h5, conversion['src_name'] + str(i), dst_h5, conversion['dst_name'] + str(i))
            elif conversion['dtype'] == 'float':
                copy_float_dataset(src_h5, conversion['src_name'] + str(i), dst_h5, conversion['dst_name'] + str(i))

    print_file_info(dst_h5, dst_file)

    src_h5.close()
    dst_h5.close()

def copy_byte_dataset(src_h5, src_ds, dst_h5, dst_ds):
    
    data = np.array(src_h5[src_ds][:].byteswap().view('uint8'), dtype=np.uint8)
    dst_h5.create_dataset(dst_ds, shape=data.shape, data=data)
    
def copy_float_dataset(src_h5, src_ds, dst_h5, dst_ds):
    
    data = np.array(src_h5[src_ds][:], dtype=np.float32)
    dst_h5.create_dataset(dst_ds, shape=data.shape, data=data)

