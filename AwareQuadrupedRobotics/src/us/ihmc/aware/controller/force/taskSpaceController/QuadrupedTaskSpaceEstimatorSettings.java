package us.ihmc.aware.controller.force.taskSpaceController;

public class QuadrupedTaskSpaceEstimatorSettings
{
   private double lipNaturalFrequency;

   public QuadrupedTaskSpaceEstimatorSettings()
   {
      this.lipNaturalFrequency = Math.sqrt(9.81 / 1.0);
   }

   public void setLipNaturalFrequency(double lipNaturalFrequency)
   {
      this.lipNaturalFrequency = lipNaturalFrequency;
   }

   public double getLipNaturalFrequency()
   {
      return lipNaturalFrequency;
   }
}

