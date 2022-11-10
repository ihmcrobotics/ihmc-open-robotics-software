package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.ConstraintOptimizerParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.ConstraintOptimizerParametersReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class SteppingEnvironmentalConstraintParameters
{
   private final ConstraintOptimizerParametersReadOnly constraintOptimizerParameters = new ConstraintOptimizerParameters();

   private static final double minPlanarRegionAreaForStepping = 0.05;
   private static final double maxPlanarRegionNormalAngleForStepping = Math.toRadians(25.0);

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
}
