# Running the logger on a QNAP NAS system with video capture

It is possible to capture video on a QNAP NAS system using the Linux Station from QNAP.

## Tested QNAP QTS Version

This guide is tested with QTS 4.3.4.0435 Build 20171230 on a QNAP TVS1282T with a Decklink DUO 2 installed

## Setting up

- Install Linux Station on your QNAP with Ubuntu 16.04
- Install neccessary packages
    - sudo apt-get install libavformat-ffmpeg56 libavcodec-ffmpeg56 libswscale-ffmpeg3 libboost-thread1.58.0 openjdk-8-jre libgl1
- Download "Desktop Video 10.8.5" for Linux from [https://www.blackmagicdesign.com/support/family/capture-and-playback](https://www.blackmagicdesign.com/support/family/capture-and-playback) on the Linux Station installation
- Unpack Desktop Video 10.8.5
- Install debian packages: `sudo dpkg -i Blackmagic_Desktop_Video_Linux_10.8.5a4/deb/amd64/desktopvideo_10.8.5a4_amd64.deb`
- Copy insertBlackmagicDrivers.sh to the Linux Station installation
- Edit insertBlackmagicDrivers.sh and set host IP, user and password correctly

## Loading the drivers

Run insertBlackmagicDrivers.sh to copy the kernel modules to the host system and load the drivers. This file has to be run on every boot.

NOTE: If you are using a card different than the Decklink DUO 2, it will probably hang. However it will show the list of devices on the QNAP host. Adjust the mknod lines to mimick the configuration on the host and adjust the BlackmagicPreferencesStartup calls to run only for the available devices.



