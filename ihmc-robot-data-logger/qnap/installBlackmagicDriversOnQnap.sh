#!/bin/bash

if [[ $EUID -ne 0 ]]; then
	echo "This script must be run as root"
	exit 1
fi

echo "Host IP (qnap default: 10.0.5.1)"
read -e -i "10.0.5.1" hostname
echo "Host user name (qnap default: admin)"
read -e -i "admin" username
echo "Host password"
read password

echo "Directory containing Blackmagic_Desktop_Video_Linux_10.8.5.tar.gz"
read -e -i "/home/${username}/Downloads" blackmagicData

mkdir /etc/ihmc

echo "host=${hostname}" > /etc/ihmc/qnapConfig
echo "user=${username}" >> /etc/ihmc/qnapConfig
echo "password=${password}" >> /etc/ihmc/qnapConfig

apt-get install -y libavformat-ffmpeg56 libavcodec-ffmpeg56 libswscale-ffmpeg3 libboost-thread1.58.0 openjdk-8-jre libgl1

mkdir /tmp/bmd
tar xzvf ${blackmagicData}/Blackmagic_Desktop_Video_Linux_10.8.5.tar.gz -C /tmp/bmd/
dpkg -i /tmp/bmd/Blackmagic_Desktop_Video_Linux_10.8.5a4/deb/amd64/desktopvideo_10.8.5a4_amd64.deb

mkdir /etc/ihmc/init
cp `dirname $0`/insertBlackmagicDrivers.sh /etc/ihmc/init
chmod 700 /etc/ihmc/init/insertBlackmagicDrivers.sh
cp `dirname $0`/load-blackmagic-driver.service /etc/systemd/system

systemctl enable load-blackmagic-driver
