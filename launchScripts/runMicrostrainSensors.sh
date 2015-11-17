#!/bin/sh

cd `dirname $0`
pkill MicrostrainMip

./MicrostrainCalibrator -d /dev/serial/by-id/usb-Lord_Microstrain_Lord_Inertial_Sensor_0000__6233.47092-if00
./MicrostrainCalibrator -d /dev/serial/by-id/usb-Lord_Microstrain_Lord_Inertial_Sensor_0000__6233.47094-if00

cp MicrostrainMipServer MicrostrainMipServer.run

./MicrostrainMipServer.run -d /dev/serial/by-id/usb-Lord_Microstrain_Lord_Inertial_Sensor_0000__6233.47092-if00 -h 10.7.4.221 -p 50092 2> /dev/null &
./MicrostrainMipServer.run -d /dev/serial/by-id/usb-Lord_Microstrain_Lord_Inertial_Sensor_0000__6233.47094-if00 -h 10.7.4.221 -p 50094 2> /dev/null &