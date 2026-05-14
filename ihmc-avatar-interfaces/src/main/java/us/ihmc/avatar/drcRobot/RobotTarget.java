package us.ihmc.avatar.drcRobot;

public enum RobotTarget
{
   SCS, GAZEBO, REAL_ROBOT, HEAD_ON_A_STICK, DISPLAY;

   private boolean offSupport = false;

   public void setIsRobotOffSupport(boolean offSupport)
   {
      this.offSupport = offSupport;
   }

   public boolean getIsRobotOffSupport()
   {
      return offSupport;
   }
}