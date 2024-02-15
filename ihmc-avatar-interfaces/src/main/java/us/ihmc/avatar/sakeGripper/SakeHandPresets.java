package us.ihmc.avatar.sakeGripper;

/**
 * Preset position and torque settings for the Sake gripper.
 * -1.0 indicates "unspecified", such that when commanded the hand
 * keeps its previous value.
 *
 * RESET does not change goal position; The hand remains in same position.
 */
public enum SakeHandPresets
{
   /** Calibration before usage */
   CALIBRATE(0.0, 0.3),
   /** Resets error message */
   RESET(-1.0, 0.0),
   /** Fully opens fingers */
   FULLY_OPEN(1.0, 0.3),
   /** Moves fingers to closed position. Not for gripping */
   CLOSE(0.1, 0.3),
   /** Removes torque */
   RELEASE(-1.0, 0.0),
   /** Close with specified torque */
   GRIP(0.0, 0.3),
   /** Close with maximum torque */
   GRIP_HARD(0.0, 1.0),
   /** Open the fingers to 105 degrees. */
   OPEN(0.5, 0.3);

   private final double normalizedHandOpenAngle;
   private final double normalizedTorqueLimit;

   SakeHandPresets(double normalizedHandOpenAngle, double normalizedTorqueLimit)
   {
      this.normalizedHandOpenAngle = normalizedHandOpenAngle;
      this.normalizedTorqueLimit = normalizedTorqueLimit;
   }

   public final static SakeHandPresets[] values = values();

   public double getNormalizedHandOpenAngle()
   {
      return normalizedHandOpenAngle;
   }

   public double getNormalizedTorqueLimit()
   {
      return normalizedTorqueLimit;
   }
}
