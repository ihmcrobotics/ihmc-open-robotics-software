#!/bin/sh

java -XX:+UseSerialGC -Xmx14g -Xms14g -XX:NewSize=12g -XX:MaxNewSize=12g -XX:CompileThreshold=1000 -verbose:gc -Djava.library.path=lib/ -cp AcsellController.jar $@
