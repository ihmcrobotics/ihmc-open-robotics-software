package us.ihmc.avatar.joystickBasedJavaFXController;

import us.ihmc.robotics.robotSide.RobotSide;

public interface HumanoidRobotPunchMessenger
{
   void sendArmHomeConfiguration(double trajectoryDuration, RobotSide... robotSides);

   void sendArmStraightConfiguration(double trajectoryDuration, RobotSide robotSide);
}
