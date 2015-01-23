package us.ihmc.convexOptimization.randomSearch;

import us.ihmc.convexOptimization.ConvexOptimizationAdapter;
import us.ihmc.convexOptimization.ConvexOptimizationAdapterTest;


public class RandomSearchConvexOptimizationAdapterTest extends ConvexOptimizationAdapterTest
{

   public ConvexOptimizationAdapter createConvexOptimizationAdapter()
   {
      return new RandomSearchConvexOptimizationAdapter();
   }

}
