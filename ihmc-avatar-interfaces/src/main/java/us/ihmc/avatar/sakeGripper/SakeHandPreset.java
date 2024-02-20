package us.ihmc.avatar.sakeGripper;

/**
 * Presets for the Sake gripper.
 */
public enum SakeHandPreset
{
   /** Open the fingers to 105 degrees. */
   OPEN(105.0, SakeHandParameters.FINGERTIP_GRIP_FORCE_SAFE),
   /** Close almost all the way to achieve no torque when nothings being grabbed. */
   CLOSE(6.0, SakeHandParameters.FINGERTIP_GRIP_FORCE_SAFE),
   /** Close with specified torque */
   GRIP(0.0, SakeHandParameters.FINGERTIP_GRIP_FORCE_SAFE),
   /** Fully opens fingers */
   FULLY_OPEN(210.0, SakeHandParameters.FINGERTIP_GRIP_FORCE_SAFE),
   /** Close with maximum torque */
   GRIP_HARD(0.0, SakeHandParameters.FINGERTIP_GRIP_FORCE_HIGH_THRESHOLD),
   /** Moves fingers to be parallel. Not for gripping */
   PARALLEL_FINGERS(21.0, SakeHandParameters.FINGERTIP_GRIP_FORCE_SAFE),
   ;

   private final double handOpenAngle;
   private final double fingertipGripForceLimit;

   SakeHandPreset(double handOpenAngleDegrees, double fingertipGripForceLimit)
   {
      this.handOpenAngle = Math.toRadians(handOpenAngleDegrees);
      this.fingertipGripForceLimit = fingertipGripForceLimit;
   }

   public final static SakeHandPreset[] values = values();

   public double getHandOpenAngle()
   {
      return handOpenAngle;
   }

   public double getFingertipGripForceLimit()
   {
      return fingertipGripForceLimit;
   }

   public String getPascalCasedName()
   {
      return switch (this)
      {
         case OPEN -> "Open";
         case CLOSE -> "Close";
         case GRIP -> "Grip";
         case FULLY_OPEN -> "Fully Open";
         case PARALLEL_FINGERS -> "Parallel Fingers";
         case GRIP_HARD -> "Grip Hard";
      };
   }
}
