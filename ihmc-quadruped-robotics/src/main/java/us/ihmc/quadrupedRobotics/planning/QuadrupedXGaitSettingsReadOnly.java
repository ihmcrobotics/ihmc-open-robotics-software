package us.ihmc.quadrupedRobotics.planning;

import us.ihmc.commons.MathTools;

public interface QuadrupedXGaitSettingsReadOnly
{
   /**
    * Nominal x offset between front and hind feet (in meters).
    */
   double getStanceLength();

   /**
    * Nominal y offset between left and right feet (in meters).
    */
   double getStanceWidth();

   /**
    * Ground clearance for each step (in meters).
    */
   double getStepGroundClearance();

   /**
    * Time duration of each swing phase (in seconds).
    */
   double getStepDuration();

   /**
    * Time duration that both hind or both front feet feet are in support (in seconds).
    */
   double getEndDoubleSupportDuration();

   /**
    * Nominal phase shift between front and hind steps (in degrees, 0: pace, 90: amble, 180: trot).
    */
   double getEndPhaseShift();

   default boolean epsilonEquals(QuadrupedXGaitSettingsReadOnly other, double epsilon)
   {
      boolean equals = MathTools.epsilonEquals(this.getStanceLength(), other.getStanceLength(), epsilon);
      equals &= MathTools.epsilonEquals(this.getStanceWidth(), other.getStanceWidth(), epsilon);
      equals &= MathTools.epsilonEquals(this.getStepGroundClearance(), other.getStepGroundClearance(), epsilon);
      equals &= MathTools.epsilonEquals(this.getStepDuration(), other.getStepDuration(), epsilon);
      equals &= MathTools.epsilonEquals(this.getEndDoubleSupportDuration(), other.getEndDoubleSupportDuration(), epsilon);
      equals &= MathTools.epsilonEquals(this.getEndPhaseShift(), other.getEndPhaseShift(), epsilon);
      return equals;
   }
}
