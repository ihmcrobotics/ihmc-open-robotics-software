package us.ihmc.darpaRoboticsChallenge.outputs;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFPerfectSimulatedOutputWriter;
import us.ihmc.SdfLoader.SDFRobot;

public class DRCSimulationOutputWriter extends SDFPerfectSimulatedOutputWriter implements DRCOutputWriter
{

   public DRCSimulationOutputWriter(SDFRobot robot)
   {
      super(robot);
   }

   public void writeAfterController()
   {
      // Do not write here, because it will set the robot's torques while the simulation is running
   }

   public void writeAfterEstimator()
   {
      // Nothing to do here
   }

   public void writeAfterSimulationTick()
   {
      write();
   }

   public void setFullRobotModel(SDFFullRobotModel fullRobotModel)
   {
      super.setFullRobotModel(fullRobotModel);
   }

}
