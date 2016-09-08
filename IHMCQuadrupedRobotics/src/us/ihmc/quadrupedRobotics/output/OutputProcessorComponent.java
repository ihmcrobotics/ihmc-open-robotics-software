package us.ihmc.quadrupedRobotics.output;

import us.ihmc.SdfLoader.models.FullRobotModel;

public interface OutputProcessorComponent
{
   void setFullRobotModel(FullRobotModel fullRobotModel);

   void initialize();

   void update();
}
