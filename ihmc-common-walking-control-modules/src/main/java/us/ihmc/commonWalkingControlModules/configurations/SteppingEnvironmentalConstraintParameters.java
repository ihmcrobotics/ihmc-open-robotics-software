package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.ConstraintOptimizerParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.ConstraintOptimizerParametersReadOnly;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class SteppingEnvironmentalConstraintParameters
{
   private final ConstraintOptimizerParametersReadOnly constraintOptimizerParameters = new ConstraintOptimizerParameters();

   private static final double minPlanarRegionAreaForStepping = 0.05;
   private static final double maxPlanarRegionNormalAngleForStepping = Math.toRadians(25.0);
   private static final double distanceFromStanceToTrustEnvironment = 0.2;
   private static final double smallIntersectionAreaToFilter = MathTools.square(0.03);

   public ConstraintOptimizerParametersReadOnly getConstraintOptimizerParameters()
   {
      return constraintOptimizerParameters;
   }

   public double getMinPlanarRegionAreaForStepping()
   {
      return minPlanarRegionAreaForStepping;
   }

   public double getMaxPlanarRegionNormalAngleForStepping()
   {
      return maxPlanarRegionNormalAngleForStepping;
   }

   public double getSmallIntersectionAreaToFilter()
   {
      return smallIntersectionAreaToFilter;
   }

   public double getDistanceFromStanceToTrustEnvironment()
   {
      return distanceFromStanceToTrustEnvironment;
   }
}
