package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.AngularMomentumSplineType;
import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.robotModels.FullRobotModel;

public class AngularMomentumEstimationParameters
{
   /**
    * Defines the percentage of the total robot mass that is to be considered as the swing leg
    */
   private double percentageSwingLegMass = 0.05;
   /**
    * Defines the percentage of the total robot mass that is to be considered as the support leg
    */
   private double percentageSupportLegMass = 0.05;

   public double getPercentageSwingLegMass()
   {
      return percentageSwingLegMass;
   }

   public double getPercentageSupportLegMass()
   {
      return  percentageSupportLegMass;
   }

   public double getPercentageBodyMass()
   {
      return (1.0 - percentageSupportLegMass - percentageSwingLegMass);
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
