#!/bin/bash

mkdir -p $HOME/.ihmc/logs/
java -Djava.library.path=lib/ -cp ValkyrieController.jar us.ihmc.valkyrie.ValkyrieNetworkProcessor | tee $HOME/.ihmc/logs/ValkyrieNetworkProc_$(date '+%Y%m%d_%H%M%S').log
