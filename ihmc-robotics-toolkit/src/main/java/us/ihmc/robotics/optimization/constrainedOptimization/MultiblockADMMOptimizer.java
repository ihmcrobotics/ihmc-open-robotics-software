package us.ihmc.robotics.optimization.constrainedOptimization;

import org.ejml.data.DMatrixD1;
import us.ihmc.robotics.optimization.Optimizer;

/**
 * Solves a {@link MultiblockAdmmProblem} iteratively using the augmented lagrangian method
 * Each iteration updates the lagrange multipliers to better satisfy the constraints
 *
 * The optimizers used to solve each unconstrained lagrangian subproblem need to be specified separately
 */
public class MultiblockADMMOptimizer
{
   private final MultiblockAdmmProblem admm;
   private final Optimizer[] optimizers;
   private boolean verbose = true;

   public MultiblockADMMOptimizer(MultiblockAdmmProblem admm, Optimizer[] optimizers)
   {
      this.admm = admm;
      this.optimizers = optimizers;
      // TODO ensure size of optimizers is same as num blocks
   }

   public void setVerbose(boolean verbose)
   {
      this.verbose = verbose;
   }

   public DMatrixD1[] solveOverNIterations(int numLagrangeIterations, DMatrixD1[] initialValues)
   {
      // TODO ensure size of initialValues is same as num blocks

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
         System.out.println("Initial Seed");
         for (int i = 0; i < numBlocks; i++)
         {
            System.out.print(i + ": ");
            optimizers[i].printOutput();
         }
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
         // Run optimization
         for (int i = 0; i < numBlocks; i++)
         {
            initialValues[i].set(optima[i]);
            optima[i] = optimizers[i].optimize(initialValues[i]);
         }

         admm.updateLagrangeMultipliers(optima);
         admm.updateLastOptimalBlocks(optima);

         iteration += 1;

         if (verbose)
         {
            System.out.println("===== Lagrange Iteration: " + iteration + " ==========");
            for (int i = 0; i < numBlocks; i++)
            {
               System.out.print(i + ": ");
               optimizers[i].printOutput();
            }
         }

//         System.out.println("Constraint: " + blockConstraint1(optima));
      }
      return optima;
   }
}
