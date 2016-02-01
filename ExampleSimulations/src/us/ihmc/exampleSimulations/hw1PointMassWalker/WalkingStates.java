package us.ihmc.exampleSimulations.hw1PointMassWalker;

import us.ihmc.robotics.robotSide.RobotSide;

public enum WalkingStates
{
   DOUBLE_SUPPORT, LEFT_SUPPORT, RIGHT_SUPPORT;

   public RobotSide getSwingSide()
   {
      switch (this)
      {
         case RIGHT_SUPPORT:
            return RobotSide.LEFT;
         case LEFT_SUPPORT:
            return RobotSide.RIGHT;
         default:
            return null;
      }
   }
      
     public static WalkingStates getSingleSuppportStateFromSwingSide(RobotSide swingSide)
     {
        switch (swingSide)
        {
           case LEFT:
              return RIGHT_SUPPORT;      
           case RIGHT:
              return LEFT_SUPPORT;  
           default:
              return null;
        }
     }
}
