#!/bin/bash
# installation
if [ $# -lt 1 ]; then
	echo "$0 <logfile>"
	exit
fi


LOGDIR=$1
LOGVIZDIR=$HOME/bin/logviz
CLASSES=$LOGVIZDIR/classes:`echo $LOGVIZDIR/LogVisualizer_lib/*jar|tr ' ' ':'`
WS=/home/unknownid/Workspaces/ValkyrieComputer/

echo "****************************************************************************************************"
if [[ $LOGDIR == *Valkyrie* ]];then
	echo "Valkyrie Robot"
	WORKDIR=$WS/ValkyrieHardwareDrivers/
	MAIN=us.ihmc.valkyrie.visualizer.ValkyrieLogVisualizer
elif [[ $LOGDIR == *Atlas* ]];then
	echo "Atlas Robot"
	WORKDIR=$WS/Atlas/
	MAIN=us.ihmc.atlas.remote.AtlasLogVisualizer
else
	echo "Unknown Robot; please edit $PWD/$0 to fix it"
	exit
fi
echo "****************************************************************************************************"
sleep 1

cd $WORKDIR
echo $PWD
echo $CLASSES
java -Xmx8g -Xms8g -Djava.library.path=$LOGVIZDIR/LogVisualizer_lib  -cp $CLASSES $MAIN $LOGDIR
