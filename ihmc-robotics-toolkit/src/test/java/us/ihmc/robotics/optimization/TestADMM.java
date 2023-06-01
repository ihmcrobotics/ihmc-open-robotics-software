package us.ihmc.robotics.optimization;

import org.ejml.data.DMatrixD1;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.mult.VectorVectorMult_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.optimization.constrainedOptimization.AugmentedLagrangeOptimizationProblem;
import us.ihmc.robotics.optimization.constrainedOptimization.MultiblockAdmmProblem;

import static us.ihmc.robotics.Assert.assertTrue;

public class TestADMM
{
   // -------- This is the first isolated problem --------------

   private int numLagrangeIterations = 15;
   private double initialPenalty = 0.5;
   private double penaltyIncreaseFactor = 1.1;

   DMatrixD1 initial1 = new DMatrixRMaj(new double[] {4.0});
   DMatrixD1 initial2 = new DMatrixRMaj(new double[] {0.0});
   DMatrixD1[] initialValues = {initial1, initial2};


   // optimum is (0,0,0...)
   public static double costFunction(DMatrixD1 inputs)
   {
      return VectorVectorMult_DDRM.innerProd(inputs, inputs);
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

   // x[0] >= 1
   public static double constraint3(DMatrixD1 inputs)
   {
      return inputs.get(0) - 1;
   }

   public static double blockConstraint1(DMatrixD1... blocks)
   {
      return (blocks[0].get(0) + blocks[1].get(0) - 4);
   }

   @Test
   public void test()
   {
      // ======= Specify the ADMM problem =============
      AugmentedLagrangeOptimizationProblem augmentedLagrange1 = new AugmentedLagrangeOptimizationProblem(TestADMM::costFunction);
//      augmentedLagrange1.addInequalityConstraint(TestADMM::constraint3);
      AugmentedLagrangeOptimizationProblem augmentedLagrange2 = new AugmentedLagrangeOptimizationProblem(TestADMM::costFunction);
//      augmentedLagrange2.addInequalityConstraint(TestADMM::constraint3);

      MultiblockAdmmProblem admm = new MultiblockAdmmProblem();
      admm.addIsolatedProblem(augmentedLagrange1);
      admm.addIsolatedProblem(augmentedLagrange2);
      admm.addEqualityConstraint(TestADMM::blockConstraint1);
      admm.initialize(initialPenalty, penaltyIncreaseFactor);

      // ========= Everything below is for solving ==============
      int numBlocks = admm.getNumBlocks();
      Optimizer[] optimizers = new Optimizer[numBlocks];
      DMatrixD1[] optima = new DMatrixD1[numBlocks];

      for (int i = 0; i < numBlocks; i++)
      {
         optimizers[i] = new WrappedGradientDescent();
         optima[i] = new DMatrixRMaj();
      }

      // Seed the admm optimization
      System.out.println("Initial Seed");
      for (int i = 0; i < numBlocks; i++)
      {
         AugmentedLagrangeOptimizationProblem problem = admm.getIsolatedOptimizationProblems().get(i);
         optimizers[i].setCostFunction(problem.getAugmentedCostFunction());
         optima[i] = optimizers[i].optimize(initialValues[i]);

         System.out.print(i + ": ");
         optimizers[i].printOutput();
      }

      admm.updateLagrangeMultipliers(optima);
      admm.updateLastOptimalBlocks(optima);

      // Do the main lagrange loop
      int iteration = 0;
      for (int i = 0; i < numBlocks; i++)
      {
         optimizers[i].setCostFunction(admm.getAugmentedCostFunctionForBlock(i));
      }

      // Repeat lagrange step multiple times
      while (iteration < numLagrangeIterations)
      {
         System.out.println("===== Lagrange Iteration: " + iteration + " ==========");

         // Run optimization
         for (int i = 0; i < numBlocks; i++)
         {
            initialValues[i].set(optima[i]);
            optima[i] = optimizers[i].optimize(initialValues[i]);

            System.out.print(i + ": ");
            optimizers[i].printOutput();
         }

         admm.updateLagrangeMultipliers(optima);
         admm.updateLastOptimalBlocks(optima);

         iteration += 1;

         System.out.println("Constraint: " + blockConstraint1(optima));
      }

      assertTrue("x1 arrived on desired value", MathTools.epsilonCompare(optima[0].get(0), 2, 1e-3));
      assertTrue("x2 arrived on desired value", MathTools.epsilonCompare(optima[1].get(0), 2, 1e-3));
   }

}
