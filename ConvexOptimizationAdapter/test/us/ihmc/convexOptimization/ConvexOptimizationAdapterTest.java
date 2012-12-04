package us.ihmc.convexOptimization;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import us.ihmc.convexOptimization.ConvexOptimizationAdapter;

public abstract class ConvexOptimizationAdapterTest
{
   public abstract ConvexOptimizationAdapter createConvexOptimizationAdapter();
   

   @Test
   public void testASimpleCase()
   {
      // Minimize x subject to x = 2;
      ConvexOptimizationAdapter convexOptimizationAdapter = createConvexOptimizationAdapter();
      
      convexOptimizationAdapter.setLinearCostFunctionVector(new double[]{1.0});
      convexOptimizationAdapter.setLinearEqualityConstraintsAMatrix(new double[][]{{1.0}});
      convexOptimizationAdapter.setLinearEqualityConstraintsBVector(new double[]{2.0});
      
      double[] solution = convexOptimizationAdapter.solve();
      
      assertEquals(1, solution.length);
      assertEquals(2.0, solution[0], 1e-7);
   }
   
   @Test
   public void testAnotherSimpleCase()
   {
      // Minimize x subject to -x <= -2; (x >= 2)
      ConvexOptimizationAdapter convexOptimizationAdapter = createConvexOptimizationAdapter();
      
      convexOptimizationAdapter.setLinearCostFunctionVector(new double[]{1.0});
      convexOptimizationAdapter.setLinearInequalityConstraints(new double[][]{{-1.0}}, new double[]{-2.0});
      
      double[] solution = convexOptimizationAdapter.solve();
      
      assertEquals(1, solution.length);
      assertEquals(2.0, solution[0], 1e-7);
   }

}
