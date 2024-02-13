package us.ihmc.avatar.sakeGripper;

public class SakeHandParameters
{
   public static int MAX_ANGLE_BETWEEN_FINGERS = 210;    // When hand is fully open, fingertips form 210 degrees angle
   public static int CLOSED_FINGER_ANGLE = -3;           // Joint angle of a finger is -3 degrees when fully closed
   public static int OPEN_FINGER_ANGLE = 102;            // Joint angle of a finger is 102 degrees when fully open
   public static int MAX_TORQUE_NEWTONS = 29;            // Sake hand can produce 39N of torque (roughly approximated)
   public static double MAX_RATIO_VALUE = 1.0;           // Some values are converted to ratios of 0.0 to 1.0
   public static double MIN_RATIO_VALUE = 0.0;

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

   public static double denormalizeHandTorque(double normalizedHandTorque)
   {
      return normalizedHandTorque * MAX_TORQUE_NEWTONS;
   }

   public static double normalizeHandTorque(double handTorque)
   {
      return handTorque / MAX_TORQUE_NEWTONS;
   }
}
