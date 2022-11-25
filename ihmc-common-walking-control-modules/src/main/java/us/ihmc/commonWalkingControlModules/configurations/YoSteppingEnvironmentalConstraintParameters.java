package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.ConstraintOptimizerParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.ConstraintOptimizerParametersReadOnly;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.YoConstraintOptimizerParameters;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoSteppingEnvironmentalConstraintParameters extends SteppingEnvironmentalConstraintParameters
{
   private final ConstraintOptimizerParametersReadOnly constraintOptimizerParameters;

   private final YoDouble minPlanarRegionAreaForStepping;
   private final YoDouble maxPlanarRegionNormalAngleForStepping;
   private final YoDouble distanceFromStanceToTrustEnvironment;
   private final YoDouble smallIntersectionAreaToFilter;
   private final YoBoolean useSimpleSnapping;

   public YoSteppingEnvironmentalConstraintParameters(SteppingEnvironmentalConstraintParameters parameters, YoRegistry parentRegistry)
   {
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      constraintOptimizerParameters = new YoConstraintOptimizerParameters(parameters.getConstraintOptimizerParameters(), registry);
      minPlanarRegionAreaForStepping = new YoDouble("minPlanarRegionAreaForStepping", registry);
      maxPlanarRegionNormalAngleForStepping = new YoDouble("maxPlanarRegionNormalAngleForStepping", registry);
      distanceFromStanceToTrustEnvironment = new YoDouble("distanceFromStanceToTrustEnvironment", registry);
      smallIntersectionAreaToFilter = new YoDouble("smallIntersectionAreaToFilter", registry);
      useSimpleSnapping = new YoBoolean("useSimpleSnapping", registry);

      minPlanarRegionAreaForStepping.set(parameters.getMinPlanarRegionAreaForStepping());
      maxPlanarRegionNormalAngleForStepping.set(parameters.getMaxPlanarRegionNormalAngleForStepping());
      distanceFromStanceToTrustEnvironment.set(parameters.getDistanceFromStanceToTrustEnvironment());
      smallIntersectionAreaToFilter.set(parameters.getSmallIntersectionAreaToFilter());
      useSimpleSnapping.set(parameters.useSimpleSnapping());

      parentRegistry.addChild(registry);
   }

   @Override
   public ConstraintOptimizerParametersReadOnly getConstraintOptimizerParameters()
   {
      return constraintOptimizerParameters;
   }

   @Override
   public double getMinPlanarRegionAreaForStepping()
   {
      return minPlanarRegionAreaForStepping.getDoubleValue();
   }

   @Override
   public double getMaxPlanarRegionNormalAngleForStepping()
   {
      return maxPlanarRegionNormalAngleForStepping.getDoubleValue();
   }

   @Override
   public double getSmallIntersectionAreaToFilter()
   {
      return smallIntersectionAreaToFilter.getDoubleValue();
   }

   @Override
   public double getDistanceFromStanceToTrustEnvironment()
   {
      return distanceFromStanceToTrustEnvironment.getDoubleValue();
   }

   @Override
   public boolean useSimpleSnapping()
   {
      return useSimpleSnapping.getValue();
   }
}
