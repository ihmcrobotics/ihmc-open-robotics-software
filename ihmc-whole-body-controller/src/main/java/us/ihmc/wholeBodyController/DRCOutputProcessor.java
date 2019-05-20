package us.ihmc.wholeBodyController;

import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public interface DRCOutputProcessor
{
   public abstract void initialize();
   
   public abstract void processAfterController(long timestamp);

   // TODO: Remove full robot model as it is only used to carry the estimated state. Instead use the context!
   public abstract void setLowLevelControllerCoreOutput(FullHumanoidRobotModel controllerRobotModel, JointDesiredOutputListBasics lowLevelControllerCoreOutput);

   public abstract void setForceSensorDataHolderForController(ForceSensorDataHolderReadOnly forceSensorDataHolderForController);
     
   public abstract YoVariableRegistry getControllerYoVariableRegistry();
}
