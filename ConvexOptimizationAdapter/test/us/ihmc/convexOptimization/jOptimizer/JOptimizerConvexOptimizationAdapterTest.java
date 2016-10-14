package us.ihmc.convexOptimization.jOptimizer;

import us.ihmc.convexOptimization.ConvexOptimizationAdapter;
import us.ihmc.convexOptimization.ConvexOptimizationAdapterTest;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;


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
