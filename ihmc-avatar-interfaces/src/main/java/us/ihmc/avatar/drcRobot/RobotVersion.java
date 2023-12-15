package us.ihmc.avatar.drcRobot;

import us.ihmc.robotics.robotSide.RobotSide;

public interface RobotVersion
{
   default boolean hasArm(RobotSide robotSide)
   {
      return true;
   }

   default boolean hasBothArms()
   {
      return hasArm(RobotSide.LEFT) && hasArm(RobotSide.RIGHT);
   }
}
