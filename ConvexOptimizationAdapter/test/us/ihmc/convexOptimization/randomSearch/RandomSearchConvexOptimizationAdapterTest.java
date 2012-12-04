package us.ihmc.convexOptimization.randomSearch;

import us.ihmc.convexOptimization.ConvexOptimizationAdapter;
import us.ihmc.convexOptimization.ConvexOptimizationAdapterTest;
import us.ihmc.convexOptimization.randomSearch.RandomSearchConvexOptimizationAdapter;


public class RandomSearchConvexOptimizationAdapterTest extends ConvexOptimizationAdapterTest
{

   public ConvexOptimizationAdapter createConvexOptimizationAdapter()
   {
      return new RandomSearchConvexOptimizationAdapter();
   }

}
