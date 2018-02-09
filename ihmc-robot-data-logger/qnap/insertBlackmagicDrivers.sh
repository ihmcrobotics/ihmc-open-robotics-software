#!/bin/bash

. /etc/ihmc/qnapConfig

until ping -w1 -c1 ${host} &>/dev/null; do :; done

# This block copies the drivers to the host system and loads them
sshpass -p "${password}" scp -o StrictHostKeyChecking=no /lib/modules/`uname -r`/updates/dkms/blackmagic.ko /lib/modules/`uname -r`/updates/dkms/blackmagic-io.ko ${user}@${host}:/lib/modules/`uname -r`
sshpass -p "${password}" ssh -o StrictHostKeyChecking=no ${user}@${host} depmod
sshpass -p "${password}" ssh -o StrictHostKeyChecking=no ${user}@${host} modprobe blackmagic
sshpass -p "${password}" ssh -o StrictHostKeyChecking=no ${user}@${host} modprobe blackmagic-io

sshpass -p "${password}" ssh -o StrictHostKeyChecking=no ${user}@${host} ls -lh /dev/blackmagic

# This creates corresponding devices in the client system. Use the output from the previous command to adjust the nodes to mimick the installed cards
mkdir /dev/blackmagic
mknod -m 666 /dev/blackmagic/io0 c 10 54
mknod -m 666 /dev/blackmagic/io1 c 10 53
mknod -m 666 /dev/blackmagic/io2 c 10 52
mknod -m 666 /dev/blackmagic/io3 c 10 51

# Run the Blackmagic startup utility for each card. Only run for installed cards, otherwise it will hang
/usr/lib/blackmagic/BlackmagicPreferencesStartup add /dev/blackmagic/io0
/usr/lib/blackmagic/BlackmagicPreferencesStartup add /dev/blackmagic/io1
/usr/lib/blackmagic/BlackmagicPreferencesStartup add /dev/blackmagic/io2
/usr/lib/blackmagic/BlackmagicPreferencesStartup add /dev/blackmagic/io3
