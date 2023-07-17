package us.ihmc.robotics.optimization;

import org.ejml.data.DMatrixD1;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.mult.VectorVectorMult_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.optimization.constrainedOptimization.AugmentedLagrangeOptimizationProblem;
import us.ihmc.robotics.optimization.constrainedOptimization.MultiblockADMMOptimizer;
import us.ihmc.robotics.optimization.constrainedOptimization.MultiblockADMMProblem;
import us.ihmc.robotics.optimization.constrainedOptimization.ParallelizableBlockConstraintFunction;

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

   static class DoubleHolder
   {
      public double x = 0;
      public DoubleHolder(int x) {this.x = x;}
   }
   static DoubleHolder sharedDoubleHolder = new DoubleHolder(0);
   private static double blockConstraint1WithSharedObject(DMatrixD1[] blocks, int i)
   {
      sharedDoubleHolder.x = blocks[0].get(0);
      // Sleep to encourage conflict
      try
      {
         Thread.sleep(0, 1);
      }
      catch (InterruptedException e)
      {
         LogTools.info(e);
      }

      return (sharedDoubleHolder.x + blocks[1].get(0) - 4);
   }

   static DoubleHolder[] parallelizedDoubleHolders = {new DoubleHolder(0), new DoubleHolder(0)};
   private static double blockConstraint1WithParallelizedObject(DMatrixD1[] blocks, int i)
   {
      parallelizedDoubleHolders[i].x = blocks[0].get(0);
      try
      {
         Thread.sleep(0, 1);
      }
      catch (InterruptedException e)
      {
         LogTools.info(e);
      }
      return (parallelizedDoubleHolders[i].x + blocks[1].get(0) - 4);
   }

   @Test
   public void testSimpleBlockConstraint()
   {
      // ======= Specify the ADMM problem =============
      MultiblockADMMOptimizer admmOptimizer = createNewGenericUninitializedADMMOptimizer();
      admmOptimizer.getADMMProblem().initialize(initialPenalty, penaltyIncreaseFactor);
      admmOptimizer.setVerbose(true);
      int numLagrangeIterations = 15;
      DMatrixD1[] optima = admmOptimizer.solveOverNIterations(numLagrangeIterations, createNewGenericInitialSeeds());

      assertTrue("x1 arrived on desired value", MathTools.epsilonCompare(optima[0].get(0), 2, 1e-3));
      assertTrue("x2 arrived on desired value", MathTools.epsilonCompare(optima[1].get(0), 2, 1e-3));

      LogTools.debug("Test completed successfully");
   }

   @Test
   // This does not use shared resources in the costs
   public void testParallelAndSequentialAreEquivalent()
   {
      MultiblockADMMOptimizer admmOptimizerSequential = createNewGenericUninitializedADMMOptimizer();
      admmOptimizerSequential.getADMMProblem().initialize(initialPenalty, penaltyIncreaseFactor);
      admmOptimizerSequential.runSubproblemsParallel(false);
      admmOptimizerSequential.setVerbose(false);
      DMatrixD1[] optimaSequential = admmOptimizerSequential.solveOverNIterations(numLagrangeIterations, createNewGenericInitialSeeds());

      // ==================================
      MultiblockADMMOptimizer admmOptimizerParallel = createNewGenericUninitializedADMMOptimizer();
      admmOptimizerParallel.getADMMProblem().initialize(initialPenalty, penaltyIncreaseFactor);
      admmOptimizerParallel.runSubproblemsParallel(true);
      admmOptimizerParallel.setVerbose(false);
      DMatrixD1[] optimaParallel = admmOptimizerParallel.solveOverNIterations(numLagrangeIterations, createNewGenericInitialSeeds());

      assertTrue("x0 results are identical", optimaSequential[0].get(0) == optimaParallel[0].get(0));
      assertTrue("x1 reslts are identical", optimaSequential[1].get(0) == optimaParallel[1].get(0));

      LogTools.debug("Test completed successfully");
   }

   @Test
   // Using the same global constraint equations, but feeding as parallel constraint instead of normal constraint
   public void testParallelNonparallelConstraintsEqual()
   {
      MultiblockADMMOptimizer admmOptimizerNonparallel = createNewGenericUninitializedADMMOptimizer();
      admmOptimizerNonparallel.getADMMProblem().initialize(initialPenalty, penaltyIncreaseFactor);
      admmOptimizerNonparallel.setVerbose(false);
      DMatrixD1[] optimaNonparallel = admmOptimizerNonparallel.solveOverNIterations(numLagrangeIterations, createNewGenericInitialSeeds());

      MultiblockADMMOptimizer admmOptimizerParallel = createNewGenericUninitializedADMMOptimizer();
      ParallelizableBlockConstraintFunction parallelizableBlockConstraintFunction = new ParallelizableBlockConstraintFunction()
      {
         @Override
         public double calculate(DMatrixD1[] blocks, int problemIndex)
         {
            return blockConstraint1(blocks);
         }
      };
      admmOptimizerParallel.getADMMProblem().clearConstraints();
      admmOptimizerParallel.getADMMProblem().addParallelizedGlobalEqualityConstraint(parallelizableBlockConstraintFunction);
      admmOptimizerParallel.getADMMProblem().initialize(initialPenalty, penaltyIncreaseFactor);
      admmOptimizerParallel.setVerbose(false);
      DMatrixD1[] optimaParallel = admmOptimizerParallel.solveOverNIterations(numLagrangeIterations, createNewGenericInitialSeeds());

      assertTrue("x0 results are identical", optimaNonparallel[0].get(0) == optimaParallel[0].get(0));
      assertTrue("x1 reslts are identical", optimaNonparallel[1].get(0) == optimaParallel[1].get(0));
   }

   @Test
   // Check if parallelization works when the cost function uses internal objects
   // 3 cases:
   // - shared object being run sequentially
   // - shared object being run parallel
   // - unshared object being run parallel
   public void testParallelSharedResources()
   {
      Stopwatch watch = new Stopwatch();
      watch.start();
      MultiblockADMMOptimizer admmOptimizerSharedSequential = createNewGenericUninitializedADMMOptimizer();
      admmOptimizerSharedSequential.getADMMProblem().clearConstraints();
      admmOptimizerSharedSequential.getADMMProblem().addParallelizedGlobalEqualityConstraint(MultiblockADMMOptimizerTest::blockConstraint1WithSharedObject);
      admmOptimizerSharedSequential.getADMMProblem().initialize(initialPenalty, penaltyIncreaseFactor);
      admmOptimizerSharedSequential.runSubproblemsParallel(false);
      admmOptimizerSharedSequential.setVerbose(false);
      DMatrixD1[] optimaSharedSequential = admmOptimizerSharedSequential.solveOverNIterations(numLagrangeIterations, createNewGenericInitialSeeds());
      LogTools.info("Shared Sequential: " + watch.totalElapsed());
      printBlocks(optimaSharedSequential);

      watch.reset();
      MultiblockADMMOptimizer admmOptimizerSharedParallel = createNewGenericUninitializedADMMOptimizer();
      admmOptimizerSharedParallel.getADMMProblem().clearConstraints();
      admmOptimizerSharedParallel.getADMMProblem().addParallelizedGlobalEqualityConstraint(MultiblockADMMOptimizerTest::blockConstraint1WithSharedObject);
      admmOptimizerSharedParallel.getADMMProblem().initialize(initialPenalty, penaltyIncreaseFactor);
      admmOptimizerSharedParallel.runSubproblemsParallel(true);
      admmOptimizerSharedParallel.setVerbose(false);
      DMatrixD1[] optimaSharedParallel = admmOptimizerSharedParallel.solveOverNIterations(numLagrangeIterations, createNewGenericInitialSeeds());
      LogTools.info("Shared Parallel: " + watch.totalElapsed());
      printBlocks(optimaSharedParallel);

      watch.reset();
      MultiblockADMMOptimizer admmOptimizerUnsharedParallel= createNewGenericUninitializedADMMOptimizer();
      admmOptimizerUnsharedParallel.getADMMProblem().clearConstraints();
      admmOptimizerUnsharedParallel.getADMMProblem().addParallelizedGlobalEqualityConstraint(MultiblockADMMOptimizerTest::blockConstraint1WithParallelizedObject);
      admmOptimizerUnsharedParallel.getADMMProblem().initialize(initialPenalty, penaltyIncreaseFactor);
      admmOptimizerUnsharedParallel.runSubproblemsParallel(true);
      admmOptimizerUnsharedParallel.setVerbose(false);
      DMatrixD1[] optimaUnsharedParallel = admmOptimizerUnsharedParallel.solveOverNIterations(numLagrangeIterations, createNewGenericInitialSeeds());
      LogTools.info("Unshared Parallel: " + watch.totalElapsed());
      printBlocks(optimaUnsharedParallel);

      assertTrue("x0 results are identical", optimaUnsharedParallel[0].get(0) == optimaSharedSequential[0].get(0));
      assertTrue("x1 reslts are identical", optimaUnsharedParallel[1].get(0) == optimaSharedSequential[1].get(0));
   }

   public MultiblockADMMOptimizer createNewGenericUninitializedADMMOptimizer()
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
//      admm.initialize(initialPenalty, penaltyIncreaseFactor);

      // ========= Everything below is for solving ==============
      int numBlocks = admm.getNumBlocks();
      Optimizer[] optimizers = new Optimizer[numBlocks];
      for (int i = 0; i < numBlocks; i++)
      {
         WrappedGradientDescent optimizer = new WrappedGradientDescent();
         optimizer.setMaxIterations(100);
         optimizers[i] = optimizer;

      }
      MultiblockADMMOptimizer admmOptimizer = new MultiblockADMMOptimizer(admm, optimizers);
      admmOptimizer.runSubproblemsParallel(false);
      admmOptimizer.setVerbose(false);

      return admmOptimizer;
   }

   public DMatrixD1[] createNewGenericInitialSeeds()
   {
      DMatrixD1 initial1 = new DMatrixRMaj(new double[] {5.0});
      DMatrixD1 initial2 = new DMatrixRMaj(new double[] {0.0});
      DMatrixD1[] newInitialMatrix = {initial1, initial2};
      return newInitialMatrix;
   }

   public void printBlocks(DMatrixD1[] blocks)
   {
      for (DMatrixD1 block : blocks)
      {
         block.print();
      }
   }


}
