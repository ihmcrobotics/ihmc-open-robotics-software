package us.ihmc.avatar.drcRobot;

import us.ihmc.robotics.robotSide.RobotSide;

public interface RobotVersion
{
   default boolean hasHead()
   {
      return true;
   }
   
   default boolean hasArm(RobotSide robotSide)
   {
      return false;
   }

   default boolean hasBothArms()
   {
      return hasArm(RobotSide.LEFT) && hasArm(RobotSide.RIGHT);
   }

   default boolean hasHandWithFingers(RobotSide side)
   {
      return false;
   }

   default boolean hasBothHandsWithFingers()
   {
      return hasHandWithFingers(RobotSide.LEFT) && hasHandWithFingers(RobotSide.RIGHT);
   }

   // TODO remove sake gripper - use has hands
   default boolean hasSakeGripperJoints(RobotSide side)
   {
      return false;
   }
   /*TODO: should return false and should be implemented into Alexander
   *  and Unitree*/
   default boolean hasAbilityHandJoints()
   {
      return true;
   }
}
