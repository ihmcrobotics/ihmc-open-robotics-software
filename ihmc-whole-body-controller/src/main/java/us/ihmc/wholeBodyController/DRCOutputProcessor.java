package us.ihmc.wholeBodyController;

import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextJointData;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public interface DRCOutputProcessor
{
   public abstract void initialize();
   
   public abstract void processAfterController(long timestamp);
   
   public abstract void setLowLevelControllerCoreOutput(HumanoidRobotContextJointData estimatedState, JointDesiredOutputListBasics lowLevelControllerCoreOutput);

   public abstract void setForceSensorDataHolderForController(ForceSensorDataHolderReadOnly forceSensorDataHolderForController);
     
   public abstract YoVariableRegistry getControllerYoVariableRegistry();
}
