package us.ihmc.robotics.optimization;

import org.ejml.data.DMatrixD1;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.mult.VectorVectorMult_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.optimization.constrainedOptimization.AugmentedLagrangeOptimizationProblem;

import static us.ihmc.robotics.Assert.assertTrue;

/**
 * Unconstrained optimum is (0,0,0)
 * Constrained optimum is (6, 5, 0)
 */
public class TestALM
{
   private AugmentedLagrangeOptimizationProblem augmentedLagrange;
   private DMatrixD1 lagrangeInputVector = new DMatrixRMaj();
   private int numLagrangeIterations = 10;
   private double initialPenalty = 1.0;
   private double penaltyIncreaseFactor = 1.5;

   DMatrixD1 initial = new DMatrixRMaj(new double[] {10.0, 14.5, 16.0});

   // optimum is (0,0,0...)
   public static double costFunction(DMatrixD1 inputs)
   {
      double cost = VectorVectorMult_DDRM.innerProd(inputs, inputs);
      return cost;
   }

   // x[1] == 5
   public static double constraint1(DMatrixD1 inputs)
   {
      return inputs.get(1) - 5;
   }

   // x[0] >= 6
   public static double constraint2(DMatrixD1 inputs)
   {
      return inputs.get(0) - 6;
   }

   // x[2] <= 3
   public static double constraint3(DMatrixD1 inputs)
   {
      return -inputs.get(2) + 3;
   }

   // x[2] <= 3
   public static double constraint4(DMatrixD1 inputs)
   {
      return inputs.get(0) + inputs.get(1) + inputs.get(2) - 6.0;
   }

   @Test
   public void test()
   {
      // Set up the optimization problem
      augmentedLagrange = new AugmentedLagrangeOptimizationProblem(TestALM::costFunction);
      augmentedLagrange.addEqualityConstraint(TestALM::constraint1);
      augmentedLagrange.addInequalityConstraint(TestALM::constraint2);
      augmentedLagrange.addInequalityConstraint(TestALM::constraint3);
//      augmentedLagrange.addEqualityConstraint(TestALM::constraint4);
      augmentedLagrange.initialize(initialPenalty, penaltyIncreaseFactor);

      // Set up the optimizer
      Optimizer optimizer = new WrappedGradientDescent();

      // Repeat lagrange step multiple times
      int iteration = 0;
      DMatrixD1 optimumX = new DMatrixRMaj();
      optimizer.setCostFunction(augmentedLagrange.getAugmentedCostFunction());
      while (iteration < numLagrangeIterations)
      {
         System.out.println("===== Lagrange Iteration: " + iteration + "==========");

         // Run optimization
         optimumX = optimizer.optimize(initial);
         augmentedLagrange.updateLagrangeMultipliers(optimumX);
         initial.set(optimumX);

         iteration += 1;

         optimizer.printOutput();
      }
      assertTrue("x1 arrived on desired value", MathTools.epsilonCompare(optimumX.get(0), 6, 1e-3));
      assertTrue("x2 arrived on desired value", MathTools.epsilonCompare(optimumX.get(1), 5, 1e-3));
      assertTrue("x3 arrived on desired value", MathTools.epsilonCompare(optimumX.get(2), 0, 1e-3));

   }


}
