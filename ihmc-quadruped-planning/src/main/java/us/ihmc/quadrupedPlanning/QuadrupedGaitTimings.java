package us.ihmc.quadrupedPlanning;

public class QuadrupedGaitTimings implements QuadrupedGaitTimingsBasics
{
   private double maxSpeed;
   private double stepDuration;
   private double endDoubleSupportDuration;

   public QuadrupedGaitTimings()
   {
   }

   public QuadrupedGaitTimings(QuadrupedGaitTimingsReadOnly defaultGaitTimings)
   {
      set(defaultGaitTimings);
   }

   @Override
   public void setMaxSpeed(double maxSpeed)
   {
      this.maxSpeed = maxSpeed;
   }

   @Override
   public void setStepDuration(double stepDuration)
   {
      this.stepDuration = stepDuration;
   }

   @Override
   public void setEndDoubleSupportDuration(double endDoubleSupportDuration)
   {
      this.endDoubleSupportDuration = endDoubleSupportDuration;
   }

   @Override
   public double getMaxSpeed()
   {
      return maxSpeed;
   }

   @Override
   public double getStepDuration()
   {
      return stepDuration;
   }

   @Override
   public double getEndDoubleSupportDuration()
   {
      return endDoubleSupportDuration;
   }
}
