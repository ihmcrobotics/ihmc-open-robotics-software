package us.ihmc.footstepPlanning.communication;

import us.ihmc.robotics.robotSide.RobotSide;

public enum UserInterfaceIKMode
{
   LEFT_ARM, RIGHT_ARM, NECK, CHEST;

   public boolean isArmMode()
   {
      switch (this)
      {
         case LEFT_ARM:
         case RIGHT_ARM:
            return true;
         default:
            return false;
      }
   }

   public RobotSide getSide()
   {
      switch (this)
      {
         case LEFT_ARM:
            return RobotSide.LEFT;
         case RIGHT_ARM:
            return RobotSide.RIGHT;
         default:
            return null;
      }
   }
}
