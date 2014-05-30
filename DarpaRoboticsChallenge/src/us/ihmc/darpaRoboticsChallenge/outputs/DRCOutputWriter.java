package us.ihmc.darpaRoboticsChallenge.outputs;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.sensorProcessing.sensors.ForceSensorDataHolder;

public interface DRCOutputWriter
{
   public abstract void initialize();
   
   public abstract void writeAfterController(long timestamp);

   public abstract void setFullRobotModel(SDFFullRobotModel controllerModel);

   public abstract void setForceSensorDataHolderForController(ForceSensorDataHolder forceSensorDataHolderForController);
   
   public abstract YoVariableRegistry getControllerYoVariableRegistry();
}
