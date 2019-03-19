package us.ihmc.quadrupedPlanning;

public interface QuadrupedXGaitSettingsBasics extends QuadrupedXGaitSettingsReadOnly
{
   void setMaxSpeed(double maxSpeed);

   void setStanceLength(double stanceLength);

   void setStanceWidth(double stanceWidth);

   void setStepGroundClearance(double stepGroundClearance);

   void setStepDuration(double stepDuration);

   void setEndDoubleSupportDuration(double endDoubleSupportDuration);

   default void set(QuadrupedXGaitSettingsReadOnly other)
   {
      setMaxSpeed(other.getMaxSpeed());
      setStanceLength(other.getStanceLength());
      setStanceWidth(other.getStanceWidth());
      setStepGroundClearance(other.getStepGroundClearance());
      setStepDuration(other.getStepDuration());
      setEndDoubleSupportDuration(other.getEndDoubleSupportDuration());
   }
}
