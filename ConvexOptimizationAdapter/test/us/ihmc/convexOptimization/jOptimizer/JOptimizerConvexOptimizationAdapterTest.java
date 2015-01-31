package us.ihmc.convexOptimization.jOptimizer;

import us.ihmc.convexOptimization.ConvexOptimizationAdapter;
import us.ihmc.convexOptimization.ConvexOptimizationAdapterTest;



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
