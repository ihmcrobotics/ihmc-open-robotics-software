package us.ihmc.avatar.sakeGripper;

public class SakeHandParameters
{
   public static int MAX_ANGLE_BETWEEN_FINGERS = 210;          // When hand is fully open, fingertips form 210 degrees angle
   public static int CLOSED_FINGER_ANGLE = -3;                 // Joint angle of a finger is -3 degrees when fully closed
   public static int OPEN_FINGER_ANGLE = 102;                  // Joint angle of a finger is 102 degrees when fully open
   public static int FINGERTIP_GRIP_FORCE_HARDWARE_LIMIT = 29; // Sake hand can produce 29 N of grip force between the fingertips
   public static double MAX_RATIO_VALUE = 1.0;                 // Some values are converted to ratios of 0.0 to 1.0

   /**
    * @param normalizedHandOpenAngle 0.0 (closed) to 1.0 (open)
    * @return actual angle between fingers in radians
    */
   public static double denormalizeHandOpenAngle(double normalizedHandOpenAngle)
   {
      return normalizedHandOpenAngle * Math.toRadians(MAX_ANGLE_BETWEEN_FINGERS);
   }

   /**
    * @param handOpenAngle actual angle between fingers in radians
    * @return 0.0 (closed) to 1.0 (open)
    */
   public static double normalizeHandOpenAngle(double handOpenAngle)
   {
      return handOpenAngle / Math.toRadians(MAX_ANGLE_BETWEEN_FINGERS);
   }

   public static double denormalizeFingertipGripForceLimit(double normalizedFingertipGripForceLimit)
   {
      return normalizedFingertipGripForceLimit * FINGERTIP_GRIP_FORCE_HARDWARE_LIMIT;
   }

   public static double normalizeFingertipGripForceLimit(double fingertipGripForceLimit)
   {
      return fingertipGripForceLimit / FINGERTIP_GRIP_FORCE_HARDWARE_LIMIT;
   }
}
