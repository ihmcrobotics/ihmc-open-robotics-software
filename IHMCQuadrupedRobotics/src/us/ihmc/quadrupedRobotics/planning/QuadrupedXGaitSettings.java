package us.ihmc.quadrupedRobotics.planning;

public class QuadrupedXGaitSettings
{
   /**
    * Nominal x offset between front and hind feet (in meters).
    */
   private double stanceLength;

   /**
    * Nominal y offset between left and right feet (in meters).
    */
   private double stanceWidth;

   /**
    * Ground clearance for each step (in meters).
    */
   private double stepGroundClearance;

   /**
    * Time duration of each swing phase (in seconds).
    */
   private double stepDuration;

   /**
    * Time duration that both hind or both front feet feet are in support (in seconds).
    */
   private double endDoubleSupportDuration;

   /**
    * Nominal phase shift between front and hind steps (in degrees, 0: pace, 90: amble, 180: trot).
    */
   private double endPhaseShift;

   public QuadrupedXGaitSettings()
   {
      stanceLength = 1.1;
      stanceWidth = 0.25;
      stepGroundClearance = 0.1;
      stepDuration = 0.33;
      endDoubleSupportDuration = 0.0;
      endPhaseShift = 90;
   }

   public double getStanceLength()
   {
      return stanceLength;
   }

   public void setStanceLength(double stanceLength)
   {
      this.stanceLength = stanceLength;
   }

   public double getStanceWidth()
   {
      return stanceWidth;
   }

   public void setStanceWidth(double stanceWidth)
   {
      this.stanceWidth = stanceWidth;
   }

   public double getStepGroundClearance()
   {
      return stepGroundClearance;
   }

   public void setStepGroundClearance(double stepGroundClearance)
   {
      this.stepGroundClearance = stepGroundClearance;
   }

   public double getStepDuration()
   {
      return stepDuration;
   }

   public void setStepDuration(double stepDuration)
   {
      this.stepDuration = stepDuration;
   }

   public double getEndDoubleSupportDuration()
   {
      return endDoubleSupportDuration;
   }

   public void setEndDoubleSupportDuration(double endDoubleSupportDuration)
   {
      this.endDoubleSupportDuration = endDoubleSupportDuration;
   }

   public double getEndPhaseShift()
   {
      return endPhaseShift;
   }

   public void setEndPhaseShift(double xGaitPhase)
   {
      this.endPhaseShift = xGaitPhase;
   }
}
