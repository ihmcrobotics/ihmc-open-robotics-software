package us.ihmc.avatar.sakeGripper;

/**
 * Presets for the Sake gripper.
 */
public enum SakeHandPresets
{
   /** Open the fingers to 105 degrees. */
   OPEN(105.0, SakeHandParameters.FINGERTIP_GRIP_FORCE_SAFE),
   /** Fully opens fingers */
   FULLY_OPEN(210.0, SakeHandParameters.FINGERTIP_GRIP_FORCE_SAFE),
   /** Moves fingers to closed position. Not for gripping */
   CLOSE(21.0, SakeHandParameters.FINGERTIP_GRIP_FORCE_SAFE),
   /** Close with specified torque */
   GRIP(0.0, SakeHandParameters.FINGERTIP_GRIP_FORCE_SAFE),
   /** Close with maximum torque */
   GRIP_HARD(0.0, SakeHandParameters.FINGERTIP_GRIP_FORCE_HIGH_THRESHOLD),
   ;

   private final double handOpenAngle;
   private final double fingertipGripForceLimit;

   SakeHandPresets(double handOpenAngleDegrees, double fingertipGripForceLimit)
   {
      this.handOpenAngle = Math.toRadians(handOpenAngleDegrees);
      this.fingertipGripForceLimit = fingertipGripForceLimit;
   }

   public final static SakeHandPresets[] values = values();

   public double getHandOpenAngle()
   {
      return handOpenAngle;
   }

   public double getFingertipGripForceLimit()
   {
      return fingertipGripForceLimit;
   }
}
