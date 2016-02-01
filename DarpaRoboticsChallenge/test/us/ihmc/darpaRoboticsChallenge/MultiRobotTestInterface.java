package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;

public interface MultiRobotTestInterface
{
   public DRCRobotModel getRobotModel();

   public String getSimpleRobotName();
}
