#!/bin/bash


java -XX:+UseSerialGC -XX:CompileThreshold=1000 -verbose:gc -XX:+PrintGCTimeStamps -Djava.library.path=lib/ -cp LLAQuadrupedController.jar us.ihmc.llaQuadrupedController.LLAQuadrupedControllerFactoryDummyOutputDemo
