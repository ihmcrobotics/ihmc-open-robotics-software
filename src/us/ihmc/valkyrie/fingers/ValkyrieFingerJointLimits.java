package us.ihmc.valkyrie.fingers;

public class ValkyrieFingerJointLimits
{
   public static double getFullyExtensonPositionLimit(ValkyrieFingerJointName joint)
   {
      switch(joint)
      {
         case Thumb1:
            return 0.0;
         case Thumb2:
            return 0.0;
         case Thumb3:
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
   
   public static double getFullyFlexedPositionLimit(ValkyrieFingerJointName joint)
   {
      switch(joint)
      {
         case Thumb1:
            return 1.75;
         case Thumb2:
            return -2.0;
         case Thumb3:
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
