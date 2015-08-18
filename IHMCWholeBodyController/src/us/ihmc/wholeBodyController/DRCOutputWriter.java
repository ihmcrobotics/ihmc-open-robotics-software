package us.ihmc.wholeBodyController;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public interface DRCOutputWriter
{
   public abstract void initialize();
   
   public abstract void writeAfterController(long timestamp);

   public abstract void setFullRobotModel(SDFFullHumanoidRobotModel controllerModel, RawJointSensorDataHolderMap rawJointSensorDataHolderMap);

   public abstract void setForceSensorDataHolderForController(ForceSensorDataHolderReadOnly forceSensorDataHolderForController);
     
   public abstract YoVariableRegistry getControllerYoVariableRegistry();
}
