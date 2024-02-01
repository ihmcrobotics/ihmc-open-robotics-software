package us.ihmc.robotics.optimization;

import org.ejml.data.DMatrixD1;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.mult.VectorVectorMult_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.optimization.constrainedOptimization.AugmentedLagrangeOptimizationProblem;
import us.ihmc.robotics.optimization.constrainedOptimization.AugmentedLagrangeOptimizer;

import static us.ihmc.robotics.Assert.assertTrue;

/**
 * Augmented Lagrangian Multipliers is used to solve
 * problems given by:
 *    minimize f(x)
 *    st: Gi(x) == 0 for G[]
 *        Hi(x) >= 0 for H[]
 *
 * This test ensures that the algorithm works correctly
 */
public class AugmentedLagrangeOptimizerTest
{
   private AugmentedLagrangeOptimizationProblem augmentedLagrangeProblem;

   // optimum is (0,0,0...)
   private static double costFunctionQuadratic(DMatrixD1 inputs)
   {
      double cost = VectorVectorMult_DDRM.innerProd(inputs, inputs);
      return cost;
   }

   private static double costFunctionNonconvex(DMatrixD1 inputs)
   {
      double norm = Math.sqrt(VectorVectorMult_DDRM.innerProd(inputs, inputs));
      if (norm == 0)
      {
         return -5.0;
      }
      return -5.0 * Math.sin(norm) / norm;
   }

   // x[1] == 5
   private static double constraint1(DMatrixD1 inputs)
   {
      return inputs.get(1) - 5;
   }

   // x[0] >= 6
   private static double constraint2(DMatrixD1 inputs)
   {
      return inputs.get(0) - 6;
   }

   // x[2] <= 3
   private static double constraint3(DMatrixD1 inputs)
   {
      return -inputs.get(2) + 3;
   }

   // x[2] <= 3
   private static double constraint4(DMatrixD1 inputs)
   {
      return inputs.get(0) + inputs.get(1) + inputs.get(2) - 6.0;
   }

   private static double constraintNonconvex(DMatrixD1 inputs)
   {
      return inputs.get(0) + inputs.get(1) - 6.354061535;
   }

   @Test
   public void testIsolatedConstraints()
   {
      DMatrixD1 initial = new DMatrixRMaj(new double[] {10.0, 14.5, 16.0});
      int numLagrangeIterations = 10;
      double initialPenalty = 1.0;
      double penaltyIncreaseFactor = 1.5;

      // Set up the optimization problem
      augmentedLagrangeProblem = new AugmentedLagrangeOptimizationProblem(AugmentedLagrangeOptimizerTest::costFunctionQuadratic);
      augmentedLagrangeProblem.addEqualityConstraint(AugmentedLagrangeOptimizerTest::constraint1);
      augmentedLagrangeProblem.addInequalityConstraint(AugmentedLagrangeOptimizerTest::constraint2);
      augmentedLagrangeProblem.addInequalityConstraint(AugmentedLagrangeOptimizerTest::constraint3);
      augmentedLagrangeProblem.initialize(initialPenalty, penaltyIncreaseFactor);

      // Set up the optimizer
      Optimizer optimizer = new WrappedGradientDescent();
      AugmentedLagrangeOptimizer aloOptimizer = new AugmentedLagrangeOptimizer(optimizer, augmentedLagrangeProblem);
      aloOptimizer.setVerbose(true);
      DMatrixD1 optimumX = aloOptimizer.optimize(numLagrangeIterations, initial);

      assertTrue("x1 arrived on desired value", MathTools.epsilonCompare(optimumX.get(0), 6, 1e-3));
      assertTrue("x2 arrived on desired value", MathTools.epsilonCompare(optimumX.get(1), 5, 1e-3));
      assertTrue("x3 arrived on desired value", MathTools.epsilonCompare(optimumX.get(2), 0, 1e-3));
   }

   @Test
   public void testJointConstraints()
   {
      DMatrixD1 initial = new DMatrixRMaj(new double[] {10.0, 14.5, 16.0});
      int numLagrangeIterations = 10;
      double initialPenalty = 1.0;
      double penaltyIncreaseFactor = 1.5;

      // Set up the optimization problem
      augmentedLagrangeProblem = new AugmentedLagrangeOptimizationProblem(AugmentedLagrangeOptimizerTest::costFunctionQuadratic);
      augmentedLagrangeProblem.addEqualityConstraint(AugmentedLagrangeOptimizerTest::constraint4);
      augmentedLagrangeProblem.initialize(initialPenalty, penaltyIncreaseFactor);

      // Set up the optimizer
      Optimizer optimizer = new WrappedGradientDescent();
      AugmentedLagrangeOptimizer aloOptimizer = new AugmentedLagrangeOptimizer(optimizer, augmentedLagrangeProblem);
      aloOptimizer.setVerbose(true);
      DMatrixD1 optimumX = aloOptimizer.optimize(numLagrangeIterations, initial);

      assertTrue("x1 arrived on desired value", MathTools.epsilonCompare(optimumX.get(0), 2, 1e-3));
      assertTrue("x2 arrived on desired value", MathTools.epsilonCompare(optimumX.get(1), 2, 1e-3));
      assertTrue("x3 arrived on desired value", MathTools.epsilonCompare(optimumX.get(2), 2, 1e-3));
   }

   @Test
   public void testNonconvex()
   {
      DMatrixD1 initial = new DMatrixRMaj(new double[] {13, 14});
      int numLagrangeIterations = 10;
      double initialPenalty = 1;
      double penaltyIncreaseFactor = 1.5;

      // Set up the optimization problem
      augmentedLagrangeProblem = new AugmentedLagrangeOptimizationProblem(AugmentedLagrangeOptimizerTest::costFunctionNonconvex);
      augmentedLagrangeProblem.addEqualityConstraint(AugmentedLagrangeOptimizerTest::constraintNonconvex);
      augmentedLagrangeProblem.initialize(initialPenalty, penaltyIncreaseFactor);

      // Set up the optimizer
      WrappedGradientDescent optimizer = new WrappedGradientDescent();
      optimizer.setInitialStepSize(10.0);
      optimizer.setLearningRate(0.9);
      AugmentedLagrangeOptimizer aloOptimizer = new AugmentedLagrangeOptimizer(optimizer, augmentedLagrangeProblem);
      aloOptimizer.setVerbose(true);
      DMatrixD1 optimumX = aloOptimizer.optimize(numLagrangeIterations, initial);

      assertTrue("x1 arrived on desired value", MathTools.epsilonCompare(optimumX.get(0), 3.1770307678, 1e-3));
      assertTrue("x2 arrived on desired value", MathTools.epsilonCompare(optimumX.get(1), 3.1770307678, 1e-3));
   }
}
