#!/bin/bash

mkdir -p $HOME/.ihmc/logs/
java -Djava.library.path=lib/ -cp ValkyrieController.jar us.ihmc.valkyrie.joystick.ValkyrieJoystickBasedSteppingApplication | tee $HOME/.ihmc/logs/ValkyrieJoystickBasedSteppingApplication_$(date '+%Y%m%d_%H%M%S').log
