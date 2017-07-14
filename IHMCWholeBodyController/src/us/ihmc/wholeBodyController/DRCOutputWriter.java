package us.ihmc.wholeBodyController;

import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;

public interface DRCOutputWriter
{
   public abstract void initialize();
   
   public abstract void writeAfterController(long timestamp);

   public abstract void setFullRobotModel(FullHumanoidRobotModel controllerModel, RawJointSensorDataHolderMap rawJointSensorDataHolderMap);

   public abstract void setForceSensorDataHolderForController(ForceSensorDataHolderReadOnly forceSensorDataHolderForController);
     
   public abstract YoVariableRegistry getControllerYoVariableRegistry();
}
