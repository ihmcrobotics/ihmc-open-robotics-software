package us.ihmc.robotics.optimization.constrainedOptimization;

import org.ejml.data.DMatrixD1;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.optimization.Optimizer;

/**
 * Solves a {@link MultiblockADMMProblem} iteratively using the augmented lagrangian method
 * Each iteration updates the lagrange multipliers to better satisfy the constraints
 *
 * The optimizers used to solve each unconstrained lagrangian subproblem need to be specified separately
 */
public class MultiblockADMMOptimizer
{
   private final MultiblockADMMProblem admm;
   private final Optimizer[] optimizers;
   private boolean verbose = true;

   public MultiblockADMMOptimizer(MultiblockADMMProblem admm, Optimizer[] optimizers)
   {
      this.admm = admm;
      this.optimizers = optimizers;
      if (optimizers.length != admm.getNumBlocks())
         throw new RuntimeException("Not enough optimizers " + optimizers.length + " were provided for all blocks of the problem " + admm.getNumBlocks());
   }

   public void setVerbose(boolean verbose)
   {
      this.verbose = verbose;
   }

   public DMatrixD1[] solveOverNIterations(int numLagrangeIterations, DMatrixD1[] initialValues)
   {
      if (initialValues.length != admm.getNumBlocks())
         throw new RuntimeException(
               "Not enough initial values " + initialValues.length + " were provided for all blocks of the problem " + admm.getNumBlocks());

      int numBlocks = admm.getNumBlocks();
      DMatrixD1[] optima = new DMatrixD1[numBlocks];

      // Seed the admm optimization
      for (int i = 0; i < numBlocks; i++)
      {
         AugmentedLagrangeOptimizationProblem problem = admm.getIsolatedOptimizationProblems().get(i);
         optimizers[i].setCostFunction(problem.getAugmentedCostFunction());
         optima[i] = optimizers[i].optimize(initialValues[i]);
      }

      if (verbose)
      {
         LogTools.info("");
         System.out.println("===== Initial Seed ===============");
         admm.printResults(optima);
      }

      admm.updateLagrangeMultipliers(optima);
      admm.saveOptimalBlocksForLastIteration(optima);

      // Do the main lagrange loop
      int iteration = 0;
      for (int i = 0; i < numBlocks; i++)
      {
         optimizers[i].setCostFunction(admm.getAugmentedCostFunctionForBlock(i));
      }

      // Repeat lagrange step multiple times
      while (iteration < numLagrangeIterations)
      {
         // Run optimization
         for (int i = 0; i < numBlocks; i++)
         {
            optima[i] = optimizers[i].optimize(optima[i]);
         }

         admm.updateLagrangeMultipliers(optima);
         admm.saveOptimalBlocksForLastIteration(optima);

         iteration += 1;

         if (verbose)
         {
            LogTools.info("");
            System.out.println("===== Lagrange Iteration: " + iteration + " ==========");
            admm.printResults(optima);
         }
      }
      return optima;
   }
}
