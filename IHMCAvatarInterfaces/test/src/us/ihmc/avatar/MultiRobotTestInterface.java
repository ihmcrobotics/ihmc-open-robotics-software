package us.ihmc.avatar;

import us.ihmc.avatar.drcRobot.DRCRobotModel;

public interface MultiRobotTestInterface
{
   public DRCRobotModel getRobotModel();

   public String getSimpleRobotName();
}
