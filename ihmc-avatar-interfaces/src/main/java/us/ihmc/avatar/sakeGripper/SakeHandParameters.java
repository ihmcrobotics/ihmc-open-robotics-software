package us.ihmc.avatar.sakeGripper;

public class SakeHandParameters
{
   /**
    * When hand is fully open, fingertips form 210 degrees angle.
    * This corresponds to the normalized value of 1.0.
    */
   public static double MAX_DESIRED_HAND_OPEN_ANGLE_DEGREES = 210.0;
   /** Joint angle of a finger is -3 degrees when fully closed */
   public static double CLOSED_KNUCKLE_JOINT_ANGLE_DEGREES = -3.0;
   /** Joint angle of a finger is 102 degrees when fully open */
   public static double OPEN_KNUCKLE_JOINT_ANGLE_DEGREES = 102.0;
   /** Sake hand can produce 29 N of grip force between the fingertips */
   public static double FINGERTIP_GRIP_FORCE_HARDWARE_LIMIT = 29.0;
   /** This is a safe amount of force. */
   public static double FINGERTIP_GRIP_FORCE_SAFE = 8.7;
   /** This is a moderate amount of force. */
   public static double FINGERTIP_GRIP_FORCE_MODERATE_THRESHOLD = 14.5;
   /** This is a high amount of force. */
   public static double FINGERTIP_GRIP_FORCE_HIGH_THRESHOLD = 20.3;
   /** The stall torque of the dynamixel that drives the gripper. */
   public static double DYNAMIXEL_MX_64AR_STALL_TORQUE = 6.0;
   /** Rough guess how much knuckle torque corresponding to our limit above. */
   public static double MAX_KNUCKLE_TORQUE = 4.5;

   /**
    * @param normalizedHandOpenAngle 0.0 (closed) to 1.0 (open)
    * @return actual angle between fingers in radians
    */
   public static double denormalizeHandOpenAngle(double normalizedHandOpenAngle)
   {
      return normalizedHandOpenAngle * Math.toRadians(MAX_DESIRED_HAND_OPEN_ANGLE_DEGREES);
   }

   /**
    * @param handOpenAngle actual angle between fingers in radians
    * @return 0.0 (closed) to 1.0 (open)
    */
   public static double normalizeHandOpenAngle(double handOpenAngle)
   {
      return handOpenAngle / Math.toRadians(MAX_DESIRED_HAND_OPEN_ANGLE_DEGREES);
   }

   public static double denormalizeFingertipGripForceLimit(double normalizedFingertipGripForceLimit)
   {
      return normalizedFingertipGripForceLimit * FINGERTIP_GRIP_FORCE_HARDWARE_LIMIT;
   }

   public static double normalizeFingertipGripForceLimit(double fingertipGripForceLimit)
   {
      return fingertipGripForceLimit / FINGERTIP_GRIP_FORCE_HARDWARE_LIMIT;
   }

   public static double denormalizeKnuckleTorque(double knuckleTorque)
   {
      return knuckleTorque * MAX_KNUCKLE_TORQUE;
   }

   public static double normalizeKnuckleTorque(double knuckleTorque)
   {
      return knuckleTorque / MAX_KNUCKLE_TORQUE;
   }
}
