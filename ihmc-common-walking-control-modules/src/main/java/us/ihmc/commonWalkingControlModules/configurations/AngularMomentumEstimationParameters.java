package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.AMGeneration.AngularMomentumSplineType;

public class AngularMomentumEstimationParameters
{
   public double getPercentageSwingLegMass()
   {
      return 0.05;
   }

   public double getPercentageSupportLegMass()
   {
      return 0.05;
   }

   public double getPercentageBodyMass()
   {
      return (1.0 - getPercentageSupportLegMass() - getPercentageSwingLegMass());
   }

   public double getSwingFootMaxLift()
   {
      return 0.10;
   }

   public int getNumberOfPointsToSampleForTransfer()
   {
      return 2;
   }

   public int getNumberOfPointsToSampleForSwing()
   {
      return 2;
   }

   public AngularMomentumSplineType getSplineType()
   {
      return AngularMomentumSplineType.LINEAR;
   }
}
