package us.ihmc.avatar;

import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.yoVariables.registry.YoRegistry;

public interface AvatarControllerThreadInterface
{
   void run();

   YoRegistry getYoVariableRegistry();

   FullHumanoidRobotModel getFullRobotModel();

   HumanoidRobotContextData getHumanoidRobotContextData();
}
