package us.ihmc.robotics.optimization;

import org.ejml.data.DMatrixD1;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.mult.VectorVectorMult_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.optimization.constrainedOptimization.AugmentedLagrangeOptimizationProblem;
import us.ihmc.robotics.optimization.constrainedOptimization.MultiblockADMMOptimizer;
import us.ihmc.robotics.optimization.constrainedOptimization.MultiblockADMMProblem;

import static us.ihmc.robotics.Assert.assertTrue;

/**
 * ADMM (Alternating Direction Method of Multipliers) is used to solve
 * problems given by:
 *    minimize f1(x1) + f2(x2) + ...
 *    st:
 *       G1[](x1) == 0, G2[](x2) == 0, ...
 *       H1[](x1) >= 0, H2[](x2) >= 0, ...
 *
 *       J[](x1, x2,...) == 0
 *       K[](x1, x2,...) >= 0
 *
 * This test ensures that the algorithm works correctly
 */
public class MultiblockADMMOptimizerTest
{
   private int numLagrangeIterations = 15;
   private double initialPenalty = 0.5;
   private double penaltyIncreaseFactor = 1.1;

   private DMatrixD1 initial1 = new DMatrixRMaj(new double[] {5.0});
   private DMatrixD1 initial2 = new DMatrixRMaj(new double[] {0.0});
   private DMatrixD1[] initialValues = {initial1, initial2};

   // unconstrained optimum is (0,0,0...)
   private static double costFunction(DMatrixD1 inputs)
   {
      return VectorVectorMult_DDRM.innerProd(inputs, inputs);
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

   // x[0] >= 1
   private static double constraint3(DMatrixD1 inputs)
   {
      return inputs.get(0) - 1;
   }

   private static double blockConstraint1(DMatrixD1... blocks)
   {
      return (blocks[0].get(0) + blocks[1].get(0) - 4);
   }

   @Test
   public void testSimpleBlockConstraint()
   {
      // ======= Specify the ADMM problem =============
      AugmentedLagrangeOptimizationProblem augmentedLagrange1 = new AugmentedLagrangeOptimizationProblem(MultiblockADMMOptimizerTest::costFunction);
      augmentedLagrange1.addInequalityConstraint(MultiblockADMMOptimizerTest::constraint3);
      AugmentedLagrangeOptimizationProblem augmentedLagrange2 = new AugmentedLagrangeOptimizationProblem(MultiblockADMMOptimizerTest::costFunction);
      augmentedLagrange2.addInequalityConstraint(MultiblockADMMOptimizerTest::constraint3);

      MultiblockADMMProblem admm = new MultiblockADMMProblem();
      admm.addIsolatedProblem(augmentedLagrange1);
      admm.addIsolatedProblem(augmentedLagrange2);
      admm.addGlobalEqualityConstraint(MultiblockADMMOptimizerTest::blockConstraint1);
      admm.initialize(initialPenalty, penaltyIncreaseFactor);

      // ========= Everything below is for solving ==============
      int numBlocks = admm.getNumBlocks();
      Optimizer[] optimizers = new Optimizer[numBlocks];
      for (int i = 0; i < numBlocks; i++)
      {
         optimizers[i] = new WrappedGradientDescent();
      }

      MultiblockADMMOptimizer admmOptimizer = new MultiblockADMMOptimizer(admm, optimizers);
      admmOptimizer.runSubproblemsParallel(true); // TODO why are the parallel and sequential optimzations giving different results
      admmOptimizer.setVerbose(true);
      DMatrixD1[] optima = admmOptimizer.solveOverNIterations(numLagrangeIterations, initialValues);

      assertTrue("x1 arrived on desired value", MathTools.epsilonCompare(optima[0].get(0), 2, 1e-3));
      assertTrue("x2 arrived on desired value", MathTools.epsilonCompare(optima[1].get(0), 2, 1e-3));

      LogTools.debug("Test completed successfully");
   }

   @Test
   public void testParallelAndSequentialAreEquivalent()
   {
      // ======= Specify the ADMM problem =============
      AugmentedLagrangeOptimizationProblem augmentedLagrange1 = new AugmentedLagrangeOptimizationProblem(MultiblockADMMOptimizerTest::costFunction);
      augmentedLagrange1.addInequalityConstraint(MultiblockADMMOptimizerTest::constraint3);
      AugmentedLagrangeOptimizationProblem augmentedLagrange2 = new AugmentedLagrangeOptimizationProblem(MultiblockADMMOptimizerTest::costFunction);
      augmentedLagrange2.addInequalityConstraint(MultiblockADMMOptimizerTest::constraint3);

      MultiblockADMMProblem admm = new MultiblockADMMProblem();
      admm.addIsolatedProblem(augmentedLagrange1);
      admm.addIsolatedProblem(augmentedLagrange2);
      admm.addGlobalEqualityConstraint(MultiblockADMMOptimizerTest::blockConstraint1);
      admm.initialize(initialPenalty, penaltyIncreaseFactor);

      // ========= Everything below is for solving ==============
      int numBlocks = admm.getNumBlocks();
      Optimizer[] optimizers = new Optimizer[numBlocks];
      for (int i = 0; i < numBlocks; i++)
      {
         optimizers[i] = new WrappedGradientDescent();
      }

      MultiblockADMMOptimizer admmOptimizerSequential = new MultiblockADMMOptimizer(admm, optimizers);
      admmOptimizerSequential.runSubproblemsParallel(false);
      admmOptimizerSequential.setVerbose(false);
      DMatrixD1[] optimaSequential = admmOptimizerSequential.solveOverNIterations(numLagrangeIterations, initialValues);


      // ==================================
      // Reset the problem
      initial1 = new DMatrixRMaj(new double[] {5.0});
      initial2 = new DMatrixRMaj(new double[] {0.0});
      DMatrixD1[] newInitialMatrix = {initial1, initial2};

      augmentedLagrange1 = new AugmentedLagrangeOptimizationProblem(MultiblockADMMOptimizerTest::costFunction);
      augmentedLagrange1.addInequalityConstraint(MultiblockADMMOptimizerTest::constraint3);
      augmentedLagrange2 = new AugmentedLagrangeOptimizationProblem(MultiblockADMMOptimizerTest::costFunction);
      augmentedLagrange2.addInequalityConstraint(MultiblockADMMOptimizerTest::constraint3);

      admm = new MultiblockADMMProblem();
      admm.addIsolatedProblem(augmentedLagrange1);
      admm.addIsolatedProblem(augmentedLagrange2);
      admm.addGlobalEqualityConstraint(MultiblockADMMOptimizerTest::blockConstraint1);
      admm.initialize(initialPenalty, penaltyIncreaseFactor);
      for (int i = 0; i < numBlocks; i++)
      {
         optimizers[i] = new WrappedGradientDescent();
      }
      MultiblockADMMOptimizer admmOptimizerParallel = new MultiblockADMMOptimizer(admm, optimizers);
      admmOptimizerParallel.runSubproblemsParallel(true); // TODO why are the parallel and sequential optimzations giving different results
      admmOptimizerParallel.setVerbose(false);
      DMatrixD1[] optimaParallel = admmOptimizerParallel.solveOverNIterations(numLagrangeIterations, newInitialMatrix);

//      LogTools.info("Sequential:\n" + optimaSequential[0].get(0) + "\n" + optimaSequential[1].get(0));
//      LogTools.info("Parallel:\n" + optimaParallel[0].get(0) + "\n" + optimaParallel[1].get(0));

      assertTrue("x0 results are identical", optimaSequential[0].get(0) == optimaParallel[0].get(0));
      assertTrue("x1 reslts are identical", optimaParallel[1].get(0) == optimaParallel[1].get(0));

      LogTools.debug("Test completed successfully");
   }
}
