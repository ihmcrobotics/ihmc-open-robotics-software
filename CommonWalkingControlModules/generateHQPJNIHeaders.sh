

javac -classpath ./classes  -d ./classes src/us/ihmc/commonWalkingControlModules/controlModules/nativeOptimization/HQPNative.java
javah -classpath ../IHMCUtilities/classes:./classes -o csrc/HQPNative.h  us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.HQPNative
