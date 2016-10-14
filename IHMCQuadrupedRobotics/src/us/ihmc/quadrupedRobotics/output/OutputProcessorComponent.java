package us.ihmc.quadrupedRobotics.output;

import us.ihmc.robotModels.FullRobotModel;

public interface OutputProcessorComponent
{
   void setFullRobotModel(FullRobotModel fullRobotModel);

   void initialize();

   void update();
}
