package us.ihmc.valkyrie.fingers;

public class ValkyrieFingerJointLimits
{
   public static double getFullyExtensonPositionLimit(ValkyrieRealRobotFingerJoint joint)
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
   
   public static double getFullyFlexedPositionLimit(ValkyrieRealRobotFingerJoint joint)
   {
      switch(joint)
      {
         case ThumbRoll:
            return 1.75;
         case Thumb:
            return -4.0;
         case Index:
            return -4.0;
         case Middle:
            return -4.0;
         case Pinky:
            return -4.0;
         default:
            throw new RuntimeException("Invalid actuatable finger joint.");
      }
   }
}
