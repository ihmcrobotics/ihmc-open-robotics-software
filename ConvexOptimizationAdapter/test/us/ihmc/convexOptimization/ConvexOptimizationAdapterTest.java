package us.ihmc.convexOptimization;

import static org.junit.Assert.assertEquals;

import org.junit.Ignore;
import org.junit.Test;

import com.joptimizer.functions.ConvexMultivariateRealFunction;
import com.joptimizer.functions.LinearMultivariateRealFunction;
import com.joptimizer.functions.QuadraticMultivariateRealFunction;

public abstract class ConvexOptimizationAdapterTest
{
   public abstract ConvexOptimizationAdapter createConvexOptimizationAdapter();
   

   @Test
   public void testASimpleEqualityCase()
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
   public void testASimpleInequalityCase()
   {
      // Minimize x subject to -x <= -2; (x >= 2)
      ConvexOptimizationAdapter convexOptimizationAdapter = createConvexOptimizationAdapter();
      
      convexOptimizationAdapter.setLinearCostFunctionVector(new double[]{1.0});
      convexOptimizationAdapter.setLinearInequalityConstraints(new double[][]{{-1.0}}, new double[]{-2.0}); //-x <= -2.0
      
      double[] solution = convexOptimizationAdapter.solve();
      
      assertEquals(1, solution.length);
      assertEquals(2.0, solution[0], 1e-7);
   }
   
   @Test
   public void testASimpleMaximizationInequalityCase()
   {
      // Minimize -x subject to x <= 5
      ConvexOptimizationAdapter convexOptimizationAdapter = createConvexOptimizationAdapter();

      convexOptimizationAdapter.setLinearCostFunctionVector(new double[]{-1.0});
      convexOptimizationAdapter.setLinearInequalityConstraints(new double[][]{{1.0}}, new double[]{5.0}); // 1.0 x <= 5.0

      double[] solution = convexOptimizationAdapter.solve();

      assertEquals(5.0, solution[0], 1e-5);
   }
   
   @Ignore // Need to implement addQuadraticInequalities
   @Test
   public void testLinearCostQuadraticInequalityOptimizationProblem()
   {
      // Minimize -x-y subject to x^2 + y^2 <= 4  (1/2 [x y] [I] [x y]^T - 2 <= 0)
      ConvexOptimizationAdapter convexOptimizationAdapter = createConvexOptimizationAdapter();
      
      convexOptimizationAdapter.setLinearCostFunctionVector(new double[]{-1.0, -1.0});

      // Quadratic Inequalities
      double[][] PMatrix = new double[][]{{1.0, 0.0}, {0.0, 1.0}};
      double[] qVector = new double[]{0.0, 0.0};
      double r = -2;
      
      convexOptimizationAdapter.addQuadraticInequalities(PMatrix, qVector, r);

      double[] solution = convexOptimizationAdapter.solve();

//      if (VERBOSE) System.out.println("solution = (" + solution[0] + ", " + solution[1] + ")");
      
      assertEquals(Math.sqrt(2.0), solution[0], 1e-5);
      assertEquals(Math.sqrt(2.0), solution[1], 1e-5);
   }
   
   @Test
   public void testLinearCostFullyLinearConstrainedEqualityOptimizationProblem()
   {
      // Minimize x subject to x+y=4 and x-y=2. Should return (3,1).
      ConvexOptimizationAdapter convexOptimizationAdapter = createConvexOptimizationAdapter();
      convexOptimizationAdapter.setLinearCostFunctionVector(new double[]{1.0, 0.0});

      // Equalities:
      double[][] equalityAMatrix = new double[][]{{1.0, 1.0}, {1.0, -1.0}};
      double[] equalityBVector = new double[]{4.0, 2.0};
      
      convexOptimizationAdapter.setLinearEqualityConstraintsAMatrix(equalityAMatrix);
      convexOptimizationAdapter.setLinearEqualityConstraintsBVector(equalityBVector);
      
      double[] solution = convexOptimizationAdapter.solve();

//      if (VERBOSE) System.out.println("solution = (" + solution[0] + ", " + solution[1] + ")");
      assertEquals(3.0, solution[0], 1e-5);
      assertEquals(1.0, solution[1], 1e-5);
   }
   
   @Test
   public void testZeroCostLinearEqualityOptimizationProblem() throws Exception
   {
      // Minimize 0 subject to x+y=4. Should return any feasible solution.
      ConvexOptimizationAdapter convexOptimizationAdapter = createConvexOptimizationAdapter();
      convexOptimizationAdapter.setLinearCostFunctionVector(new double[]{0.0, 0.0});

      // Equalities:
      double[][] equalityAMatrix = new double[][]{{1.0, 1.0}};
      double[] equalityBVector = new double[]{4.0};
      
      convexOptimizationAdapter.setLinearEqualityConstraintsAMatrix(equalityAMatrix);
      convexOptimizationAdapter.setLinearEqualityConstraintsBVector(equalityBVector);
      
      double[] solution = convexOptimizationAdapter.solve();

//      if (VERBOSE) System.out.println("solution = (" + solution[0] + ", " + solution[1] + ")");
      assertEquals(4.0, solution[0] + solution[1], 1e-5);
   }
   
   @Ignore //Not implemented yet!
   @Test
   public void testLinearCostLinearEqualityQuadraticInequalityOptimizationProblem() throws Exception
   {
      // Minimize x subject to x+y=4 and y >= x^2. Answer should be ((-1-sqrt(17))/2, (9+sqrt(17))/2))
      ConvexOptimizationAdapter convexOptimizationAdapter = createConvexOptimizationAdapter();
      convexOptimizationAdapter.setLinearCostFunctionVector(new double[]{1.0, 0.0});

      // Equalities:
      double[][] equalityAMatrix = new double[][]{{1.0, 1.0}};
      double[] equalityBVector = new double[]{4.0};
      
      convexOptimizationAdapter.setLinearEqualityConstraintsAMatrix(equalityAMatrix);
      convexOptimizationAdapter.setLinearEqualityConstraintsBVector(equalityBVector);
      
      // inequalities
      double[][] pMatrix = new double[][]{{2.0, 0.0}, {0.0, 0.0}};
      double[] qVector = new double[]{0.0, -1.0};
      double r = 0.0;
      convexOptimizationAdapter.addQuadraticInequalities(pMatrix, qVector, r);
      
      double[] solution = convexOptimizationAdapter.solve();

//      if (VERBOSE) System.out.println("solution = (" + solution[0] + ", " + solution[1] + ")");
      
      assertEquals(1.0/2.0 * (-1.0 - Math.sqrt(17.0)), solution[0], 1e-5);
      assertEquals(1.0/2.0 * (9.0 + Math.sqrt(17.0)), solution[1], 1e-5);
   }

}
