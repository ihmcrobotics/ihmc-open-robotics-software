import h5py
import numpy as np

def copy_byte_dataset(src_h5, src_ds, dst_h5, dst_ds):
    
    data = np.array(src_h5[src_ds][:].byteswap().view('uint8'), dtype=np.uint8)
    dst_h5.create_dataset(dst_ds, shape=data.shape, data=data)
    
def copy_float_dataset(src_h5, src_ds, dst_h5, dst_ds):
    
    data = np.array(src_h5[src_ds][:], dtype=np.float32)
    dst_h5.create_dataset(dst_ds, shape=data.shape, data=data)
