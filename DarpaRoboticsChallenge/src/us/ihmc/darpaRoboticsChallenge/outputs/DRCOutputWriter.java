package us.ihmc.darpaRoboticsChallenge.outputs;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.sensorProcessing.sensors.ForceSensorDataHolder;

import com.yobotics.simulationconstructionset.robotController.RobotControlElement;

public interface DRCOutputWriter extends RobotControlElement
{
   public abstract void writeAfterController(long timestamp);

   public abstract void writeAfterEstimator(long timestamp);

   public abstract void writeAfterSimulationTick();

   public abstract void setFullRobotModel(SDFFullRobotModel controllerModel);

   public abstract void setEstimatorModel(SDFFullRobotModel estimatorModel);

   public abstract void setForceSensorDataHolderForController(ForceSensorDataHolder forceSensorDataHolderForController);
}
