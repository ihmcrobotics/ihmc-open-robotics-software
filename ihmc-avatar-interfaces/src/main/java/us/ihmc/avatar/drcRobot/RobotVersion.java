package us.ihmc.avatar.drcRobot;

public interface RobotVersion
{
   default boolean hasArms()
   {
      return true;
   }
}
