package us.ihmc.convexOptimization.jOptimizer;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.convexOptimization.ConvexOptimizationAdapter;
import us.ihmc.convexOptimization.ConvexOptimizationAdapterTest;


//TODO: Get this working some day!!
@ContinuousIntegrationPlan(categories = {IntegrationCategory.EXCLUDE})
public class JOptimizerConvexOptimizationAdapterTest extends ConvexOptimizationAdapterTest
{
   public ConvexOptimizationAdapter createConvexOptimizationAdapter()
   {
      return new JOptimizerConvexOptimizationAdapter();
   }

   public double getTestErrorEpsilon()
   {
      return 1e-5;
   }

}
