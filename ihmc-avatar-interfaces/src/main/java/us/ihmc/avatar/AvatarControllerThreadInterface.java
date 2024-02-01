package us.ihmc.avatar;

import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;

public interface AvatarControllerThreadInterface extends SCS2YoGraphicHolder
{
   void run();

   YoRegistry getYoVariableRegistry();

   FullHumanoidRobotModel getFullRobotModel();

   HumanoidRobotContextData getHumanoidRobotContextData();

   @Override
   YoGraphicGroupDefinition getSCS2YoGraphics();
}
