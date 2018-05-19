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


## Setting up file shares for logger

In the QNAP administrator console, enable NFS v4 file server. Create a shared folder, and add the following line to /etc/fstab

```
10.0.5.1:/[Your folder name]     /media/remote/robotLogs nfs auto        0       0
```

Create the folders and mount
```
sudo mkdir -p /media/remote/robotLogs
sudo mount -a
mkdir /media/remote/robotLogs/incoming
ln -s /media/remote/robotLogs/incoming ~/robotLogs

``` 

## Autostarting the logger
Follow the instructions in the systemd directory in ihmc-robot-data-logger. By default, the desired user is `admin` and group `administrator`.
