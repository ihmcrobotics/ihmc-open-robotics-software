package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import us.ihmc.commonWalkingControlModules.modelPredictiveController.CoMTrajectoryModelPredictiveController;

public class AngleTrackingCommand
{
   private int startIndex;

   private double startValue;
   private double startRate;
   private double finalValue;
   private double finalRate;

   private double duration;

   private double omega;

   private double weight = CoMTrajectoryModelPredictiveController.orientationTrackingWeight;

   public void setStartIndex(int startIndex)
   {
      this.startIndex = startIndex;
   }

   public void setStartValue(double startValue)
   {
      this.startValue = startValue;
   }

   public void setStartRate(double startRate)
   {
      this.startRate = startRate;
   }

   public void setFinalValue(double finalValue)
   {
      this.finalValue = finalValue;
   }

   public void setFinalRate(double finalRate)
   {
      this.finalRate = finalRate;
   }

   public void setDuration(double duration)
   {
      this.duration = duration;
   }

   public void setWeight(double weight)
   {
      this.weight = weight;
   }

   public void setOmega(double omega)
   {
      this.omega = omega;
   }

   public double getStartValue()
   {
      return startValue;
   }

   public double getStartRate()
   {
      return startRate;
   }

   public double getFinalValue()
   {
      return finalValue;
   }

   public double getFinalRate()
   {
      return finalRate;
   }

   public int getStartIndex()
   {
      return startIndex;
   }

   public double getDuration()
   {
      return duration;
   }

   public double getWeight()
   {
      return weight;
   }

   public double getOmega()
   {
      return omega;
   }
}
