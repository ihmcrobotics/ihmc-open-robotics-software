# IHMC Perception Logs (plogs)

## Python Library for HDF5 Based Perception and Robotics Logs

<br>

This library provides a convenient python based command-line tool for replaying, plotting, analyzing and viewing perception logs collected on robots using their sensor signals. 

### Installation

Install the python scripts into the binaries of your system using the install script inside plogs (navigate to the plogs directory):

    bash install.sh

### Dependencies
    h5py
    matplotlib
    python3-opencv
    seaborn

### Features and Usage

NOTE: By default, the plogs command will look for log files in the `~/.ihmc/logs/perception/`. 
To use a different path use the `--path` argument. For more details, see the `plogs.py` script.

<br>
List groups and datasets inside any HDF5 log file using:

    Command:
        
        plogs --info 20230226_192147_PerceptionLog.hdf5

    Output:

        ------------------------------------------ HDF5 File Info --------------------------------------------


        File: 20230226_192147_PerceptionLog.hdf5

        Group                                         Total Groups         Total Datasets            Data Type 

            l515/                                       2                    0                         none      
            l515/depth/                                 0                    4120                      uint8     
            l515/sensor/                                3                    0                         none      
            l515/sensor/orientation/                    0                    411                       float32   
            l515/sensor/position/                       0                    411                       float32   
            l515/sensor/time/                           0                    0                         none      
            mocap/                                      1                    0                         none      
            mocap/rigid_body/                           2                    0                         none      
            mocap/rigid_body/orientation/               0                    411                       float32   
            mocap/rigid_body/position/                  0                    411                       float32   
        ------------------------------------------------------------------------------------------------------


List groups and datasets inside any HDF5 log file using:

    Command:
    
        plogs --list

    Output:

        ------------------------------------------ HDF5 File Sizes --------------------------------------------


        File Name                                     File Size (MB)      

            20221212_184940_PerceptionLog.hdf5          617.821  MB
            20221215_234244_PerceptionLog.hdf5          119.518  MB
            20221215_234308_PerceptionLog.hdf5          60.556   MB
            20221222_141507_Ouster.hdf5                 6.771    MB
            20230102_152006_PerceptionLog.hdf5          57.530   MB

Playback any file using OpenCV based image displays using:

    plogs --play <filename>.hdf5


Plot signals using Matplotlib using: 

    plogs --plot <filename>.hdf5

Convert log files with old int32 based depth maps into new uint8 based depth maps using:

    plogs --fix <filename>.hdf5 --dst <dst_filename>.hdf5
