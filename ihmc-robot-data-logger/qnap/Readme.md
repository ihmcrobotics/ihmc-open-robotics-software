# Running the logger on a QNAP NAS system with video capture

It is possible to capture video on a QNAP NAS system using the Linux Station from QNAP.

## Tested QNAP QTS Version

This guide is tested with QTS 4.3.4.0435 Build 20171230 on a QNAP TVS1282T with a Decklink DUO 2 installed

## Setting up

- Install Linux Station on your QNAP with Ubuntu 16.04
- Copy all files in this directory to your Linux Station
- Download "Desktop Video 10.8.5" for Linux from [https://www.blackmagicdesign.com/support/family/capture-and-playback](https://www.blackmagicdesign.com/support/family/capture-and-playback) on the Linux Station installation
- Run installBlackmagicDriversonQnap.sh to setup loading drivers on boot. Enter correct values.


## Adjusting for different Blackmagic Cards


If you are using a card different than the Decklink DUO 2, it will probably not work. 

Run "ssh admin@10.0.5.1 ls -lh /dev/blackmagic" to list all blackmagic devices (adjust admin@10.0.5.1 accordingly). Edit /etc/ihmc/init/insertBlackmagicDrivers.sh to make the correct devices using mknod and adjust the BlackmagicPreferencesStartup calls to run only for the available devices.


