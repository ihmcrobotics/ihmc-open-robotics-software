package us.ihmc.avatar.sakeGripper;

import controller_msgs.msg.dds.SakeHandDesiredCommandMessage;

public class SakeHandParameters
{
   /**
    * When hand is fully open, fingertips form 210 degrees angle.
    * This corresponds to the normalized value of 1.0.
    */
   public static final double MAX_DESIRED_HAND_OPEN_ANGLE_DEGREES = 210.0;
   /** Joint angle of a knuckle when fully open */
   public static final double OPEN_KNUCKLE_JOINT_ANGLE_DEGREES = 105.0;
   /** Sake hand can produce 29 N of grip force between the fingertips */
   public static final double FINGERTIP_GRIP_FORCE_HARDWARE_LIMIT = 29.0;
   /** This is a safe amount of force. */
   public static final double FINGERTIP_GRIP_FORCE_SAFE = 8.7;
   /** This is a moderate amount of force. */
   public static final double FINGERTIP_GRIP_FORCE_MODERATE_THRESHOLD = 14.5;
   /** This is a high amount of force. */
   public static final double FINGERTIP_GRIP_FORCE_HIGH_THRESHOLD = 20.3;
   /** The stall torque of the dynamixel that drives the gripper. */
   public static final double DYNAMIXEL_MX_64AR_STALL_TORQUE = 6.0;
   /** Rough guess how much knuckle torque corresponding to our limit above. */
   public static final double KNUCKLE_TORQUE_AT_LIMIT = 2.0;
   /** Normal hand temperature. */
   public static final double NORMAL_TEMPERATURE_CELCIUS = 30.0;
   /** Warning hand temperature. */
   public static final double WARNING_TEMPERATURE_CELCIUS = 40.0;
   /** Red colored hand temperature. */
   public static final double ERROR_TEMPERATURE_CELCIUS = 50.0;
   /** The max temperature before the hand resets. */
   public static final double TEMPERATURE_LIMIT_CELCIUS = 60.0;
   /** The temperature at which the Dynamixel will break. */
   public static final int DYNAMIXEL_FAILURE_TEMPERATURE_CELCIUS = 80;
   /** Sake encoder resolution is 4096. Convert that to rad */
   public static final double RAW_SAKE_POSITION_TO_RAD = (2.0 * Math.PI) / 4096.0;
   /**
    * Robotis provide rotation speed conversion in RPM (0.11 rpm). Convert that to rad/s
    * See: <a href="https://emanual.robotis.com/docs/en/dxl/mx/mx-64/#present-speed-38"> DYNAMIXEL Protocol 1.0: Present Speed </a>
    */
   public static final double RAW_SAKE_VELOCITY_TO_RAD_PER_SEC = 0.11 * ((2.0 * Math.PI) / 60);
   public static final double RAW_SAKE_TORQUE_TO_GRIP_FORCE = FINGERTIP_GRIP_FORCE_HARDWARE_LIMIT / 1023.0;

   /**
    * @param normalizedHandOpenAngle 1.0 (open) to 0.0 (closed)
    * @return actual angle between fingers in radians
    */
   public static double denormalizeHandOpenAngle(double normalizedHandOpenAngle)
   {
      return normalizedHandOpenAngle * Math.toRadians(MAX_DESIRED_HAND_OPEN_ANGLE_DEGREES);
   }

   /**
    * @param handOpenAngle actual angle between fingers in radians
    * @return 1.0 (open) to 0.0 (closed)
    */
   public static double normalizeHandOpenAngle(double handOpenAngle)
   {
      return handOpenAngle / Math.toRadians(MAX_DESIRED_HAND_OPEN_ANGLE_DEGREES);
   }

   public static double denormalizeHandPosition(double normalizedPosition, double positionLowerLimit, double positionUpperLimit)
   {
      return normalizedPosition * (positionUpperLimit - positionLowerLimit) + positionLowerLimit;
   }

   public static double normalizeHandPosition(double handPosition, double positionLowerLimit, double positionUpperLimit)
   {
      return (handPosition - positionLowerLimit) / (positionUpperLimit - positionLowerLimit);
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
      return knuckleTorque * KNUCKLE_TORQUE_AT_LIMIT;
   }

   public static double normalizeKnuckleTorque(double knuckleTorque)
   {
      return knuckleTorque / KNUCKLE_TORQUE_AT_LIMIT;
   }

   public static double knuckleJointAngleToHandOpenAngle(double knuckleJointAngle)
   {
      return denormalizeHandOpenAngle(knuckleJointAngle / Math.toRadians(OPEN_KNUCKLE_JOINT_ANGLE_DEGREES));
   }

   public static double knuckleJointAnglesToHandOpenAngle(double knuckle1JointAngle, double knuckle2JointAngle)
   {
      return knuckleJointAngleToHandOpenAngle(knuckle1JointAngle + knuckle2JointAngle) / 2.0;
   }

   public static double handOpenAngleToKnuckleJointAngle(double handOpenAngle)
   {
      return normalizeHandOpenAngle(handOpenAngle) * Math.toRadians(OPEN_KNUCKLE_JOINT_ANGLE_DEGREES);
   }

   public static double handOpenAngleToPosition(double handOpenAngle, double positionLowerLimit, double positionUpperLimit)
   {
      return normalizeHandOpenAngle(handOpenAngle) * (positionUpperLimit - positionLowerLimit) + positionLowerLimit;
   }

   public static double handPositionToOpenAngle(double handPosition, double positionLowerLimit, double positionUpperLimit)
   {
      return normalizeHandPosition(handPosition, positionLowerLimit, positionUpperLimit) * Math.toRadians(MAX_DESIRED_HAND_OPEN_ANGLE_DEGREES);
   }

   public static double handPositionToKnuckleJointAngle(double handPosition, double positionLowerLimit, double positionUpperLimit)
   {
      return handOpenAngleToKnuckleJointAngle(handPositionToOpenAngle(handPosition, positionLowerLimit, positionUpperLimit));
   }

   public static int gripForceToRawTorque(double gripForce) {
      return (int) (gripForce / RAW_SAKE_TORQUE_TO_GRIP_FORCE);
   }

   public static double knuckleTorqueToGripForce(double knuckleTorque)
   {
      return normalizeKnuckleTorque(knuckleTorque) * FINGERTIP_GRIP_FORCE_HARDWARE_LIMIT;
   }

   public static double gripForceToKnuckleTorque(double gripForce)
   {
      return normalizeFingertipGripForceLimit(gripForce) * KNUCKLE_TORQUE_AT_LIMIT;
   }

   public static void resetDesiredCommandMessage(SakeHandDesiredCommandMessage sakeHandDesiredCommandMessage)
   {
      sakeHandDesiredCommandMessage.setGripperDesiredPosition(Double.NaN);
      sakeHandDesiredCommandMessage.setRawGripperTorqueLimit(-1);
      sakeHandDesiredCommandMessage.setRequestCalibration(false);
      sakeHandDesiredCommandMessage.setRequestResetErrors(false);
      sakeHandDesiredCommandMessage.setTorqueOn(true);
   }
}
