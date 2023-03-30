package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.ConstraintOptimizerParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.ConstraintOptimizerParametersReadOnly;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class SteppingEnvironmentalConstraintParameters
{
   private final ConstraintOptimizerParametersReadOnly constraintOptimizerParameters;

   private static final double minPlanarRegionAreaForStepping = 0.05;
   private static final double maxPlanarRegionNormalAngleForStepping = Math.toRadians(25.0);
   private static final double distanceFromStanceToTrustEnvironment = 0.2;
   private static final double smallIntersectionAreaToFilter = MathTools.square(0.03);
   private static final boolean useSimpleSnapping = false;

   public SteppingEnvironmentalConstraintParameters()
   {
      ConstraintOptimizerParameters parametersLocal = new ConstraintOptimizerParameters();
      parametersLocal.setMaxX(0.05);
      parametersLocal.setMaxY(0.05);

      constraintOptimizerParameters = parametersLocal;
   }

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

   public boolean useSimpleSnapping()
   {
      return useSimpleSnapping;
   }
}
