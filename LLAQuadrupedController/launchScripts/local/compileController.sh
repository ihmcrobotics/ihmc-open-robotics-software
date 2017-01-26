#!/bin/bash


/opt/jet11.3-beta3-amd64/profile1.8.0_101/jre/bin/java -XX:+UseSerialGC -XX:CompileThreshold=1000 -verbose:gc -XX:+PrintGCTimeStamps -Djava.library.path=lib/ -cp LLAQuadrupedController.jar us.ihmc.llaQuadrupedController.LLAQuadrupedControllerFactoryDummyOutputDemo
