package us.ihmc.valkyrie.fingers;

import us.ihmc.robotics.robotSide.RobotSide;

public class ValkyrieFingerJointLimits
{
   public static double getFullyExtensonPositionLimit(RobotSide robotSide, ValkyrieRealRobotFingerJoint joint)
   {
      switch(joint)
      {
         case ThumbRoll:
            return 0.0;
         case Thumb:
            return 0.0;
         case Index:
            return 0.0;
         case Middle:
            return 0.0;
         case Pinky:
            return 0.0;
         default:
            throw new RuntimeException("Invalid actuatable finger joint.");
      }
   }
   
   public static double getFullyFlexedPositionLimit(RobotSide robotSide, ValkyrieRealRobotFingerJoint joint)
   {
      switch(joint)
      {
         case ThumbRoll:
            return 1.75;
         case Thumb:
            return robotSide == RobotSide.LEFT ? -4.0 : 4.0;
         case Index:
            return robotSide == RobotSide.LEFT ? -4.0 : 4.0;
         case Middle:
            return robotSide == RobotSide.LEFT ? -4.0 : 4.0;
         case Pinky:
            return robotSide == RobotSide.LEFT ? -4.0 : 4.0;
         default:
            throw new RuntimeException("Invalid actuatable finger joint.");
      }
   }
}
