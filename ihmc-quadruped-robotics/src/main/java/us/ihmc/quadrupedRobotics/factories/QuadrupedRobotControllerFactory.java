package us.ihmc.quadrupedRobotics.factories;

import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.communication.streamingData.GlobalDataProducer;

public interface QuadrupedRobotControllerFactory
{
   public RobotController createRobotController();

   public void setControlDt(double controlDt);

   public void setFullRobotModel(FullQuadrupedRobotModel fullRobotModel);

   public void setGlobalDataProducer(GlobalDataProducer globalDataProducer);
}
