#!/bin/sh

cd `dirname $0`
pkill MicrostrainMip

./MicrostrainCalibrator -d /dev/serial/by-id/usb-Lord_Microstrain_Lord_Inertial_Sensor_0000__6233.47095-if00
./MicrostrainCalibrator -d /dev/serial/by-id/usb-Lord_Microstrain_Lord_Inertial_Sensor_0000__6233.47093-if00

cp MicrostrainMipServer MicrostrainMipServer.run

./MicrostrainMipServer.run -d /dev/serial/by-id/usb-Lord_Microstrain_Lord_Inertial_Sensor_0000__6233.47095-if00 -h 10.185.0.30 -p 50095 2> /dev/null &
./MicrostrainMipServer.run -d /dev/serial/by-id/usb-Lord_Microstrain_Lord_Inertial_Sensor_0000__6233.47093-if00 -h 10.185.0.30 -p 50093 2> /dev/null &
