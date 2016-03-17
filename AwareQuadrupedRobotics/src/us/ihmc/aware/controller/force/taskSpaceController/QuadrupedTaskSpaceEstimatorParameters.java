package us.ihmc.aware.controller.force.taskSpaceController;

public class QuadrupedTaskSpaceEstimatorParameters
{
   private double dcmNaturalFrequency;

   public QuadrupedTaskSpaceEstimatorParameters()
   {
      dcmNaturalFrequency = Math.sqrt(9.81 / 1.0);
   }

   public double getDcmNaturalFrequency()
   {
      return dcmNaturalFrequency;
   }

   public void setDcmNaturalFrequency(double dcmNaturalFrequency)
   {
      this.dcmNaturalFrequency = dcmNaturalFrequency;
   }
}
