package us.ihmc.avatar.drcRobot;

import us.ihmc.robotics.robotSide.RobotSide;

public interface RobotVersion
{
   default boolean hasArm(RobotSide robotSide)
   {
      return false;
   }

   default boolean hasHead()
   {
      return true;
   }

   default boolean hasBothArms()
   {
      return hasArm(RobotSide.LEFT) && hasArm(RobotSide.RIGHT);
   }

   default boolean hasHead()
   {
      return true;
   }

   default boolean hasSakeGripperJoints(RobotSide side)
   {
      return false;
   }
}
