package us.ihmc.darpaRoboticsChallenge.outputs;

import us.ihmc.SdfLoader.SDFFullRobotModel;

import com.yobotics.simulationconstructionset.robotController.RobotControlElement;

public interface DRCOutputWriter extends RobotControlElement
{
   public abstract void writeAfterController();
   public abstract void writeAfterEstimator();
   
   public abstract void writeAfterSimulationTick();
   public abstract void setFullRobotModel(SDFFullRobotModel controllerModel);
}
