#!/bin/sh

java -XX:+UseSerialGC -Xmx7g -Xms7g -XX:NewSize=6g -XX:MaxNewSize=6g -XX:CompileThreshold=1000 -verbosegc -Djava.library.path=lib/ -cp AcsellController.jar $@
