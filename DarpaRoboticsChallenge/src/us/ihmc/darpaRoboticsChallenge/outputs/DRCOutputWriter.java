package us.ihmc.darpaRoboticsChallenge.outputs;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.sensorProcessing.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;

public interface DRCOutputWriter
{
   public abstract void initialize();
   
   public abstract void writeAfterController(long timestamp);

   public abstract void setFullRobotModel(SDFFullRobotModel controllerModel, RawJointSensorDataHolderMap rawJointSensorDataHolderMap);

   public abstract void setForceSensorDataHolderForController(ForceSensorDataHolder forceSensorDataHolderForController);
     
   public abstract YoVariableRegistry getControllerYoVariableRegistry();
}
