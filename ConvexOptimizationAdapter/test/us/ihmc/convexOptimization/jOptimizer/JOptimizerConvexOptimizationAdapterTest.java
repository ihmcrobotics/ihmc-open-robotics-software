package us.ihmc.convexOptimization.jOptimizer;

import us.ihmc.convexOptimization.ConvexOptimizationAdapter;
import us.ihmc.convexOptimization.ConvexOptimizationAdapterTest;
import us.ihmc.convexOptimization.jOptimizer.JOptimizerConvexOptimizationAdapter;



public class JOptimizerConvexOptimizationAdapterTest extends ConvexOptimizationAdapterTest
{
   public ConvexOptimizationAdapter createConvexOptimizationAdapter()
   {
      return new JOptimizerConvexOptimizationAdapter();
   }

}
