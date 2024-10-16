package us.ihmc.footstepPlanning.communication;

import us.ihmc.commons.robotics.robotSide.RobotSide;

public enum UserInterfaceIKMode
{
   LEFT_ARM, RIGHT_ARM, NECK, CHEST, LEFT_LEG, RIGHT_LEG;

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

   public boolean isLegMode()
   {
      switch (this)
      {
         case LEFT_LEG:
         case RIGHT_LEG:
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
         case LEFT_LEG:
            return RobotSide.LEFT;
         case RIGHT_ARM:
         case RIGHT_LEG:
            return RobotSide.RIGHT;
         default:
            return null;
      }
   }
}
