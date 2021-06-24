#!/bin/bash -e


NETWORK_PROCESSOR_CLASS=us.ihmc.valkyrie.ValkyrieNetworkProcessor

# Little helper function to find Network Processor instances.
function find_network_processor_pids {
     old_processes=$(pgrep -a java | grep $NETWORK_PROCESSOR_CLASS | awk '{print $1}')
}

# Nuke any existing Network Processor processes, giving them time to die
old_processes=""
find_network_processor_pids

if [[ ! -z "$old_processes" ]]; then
    for pid in "$old_processes"; do
        echo "Got Network Processor with PID $pid"
        kill $old_processes && echo Waiting for old Network Processor processes to die && sleep 3
        kill -9 $old_processes && echo Waiting for old Network Processor processes to die harder && sleep 3
    done
fi

# Check whether there's something still not dead, in which case give up
find_network_processor_pids
if [[ ! -z "$old_processes" ]]; then
    echo "Unable to kill old Network Processor PIDs $old_processes"
    exit 1
fi

echo "Starting Network Processor"

IHMC_LOGS_DIR=$HOME/.ihmc/logs
OLDEST_LOG_AGE_HOURS=3
if [[ ! -d $IHMC_LOGS_DIR ]]; then
    mkdir -p $IHMC_LOGS_DIR
fi
java -Djava.library.path=lib/ -cp ValkyrieController.jar $NETWORK_PROCESSOR_CLASS 2>&1 | tee $IHMC_LOGS_DIR/ValkyrieNetworkProc_$(date '+%Y%m%d_%H%M%S').log &
# Some time for process to get established
sleep 1
find_network_processor_pids

trap "kill $old_processes; \
      sleep 5; \
      kill -9 $old_processes; exit 0" INT TERM
wait
