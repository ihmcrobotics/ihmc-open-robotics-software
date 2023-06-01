package us.ihmc.robotics.optimization.constrainedOptimization;

import org.ejml.data.DMatrixD1;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.robotics.optimization.Optimizer;

public class AugmentedLagrangeOptimizer
{
   private final AugmentedLagrangeOptimizationProblem problem;
   private final CostFunction lagrangeCostFunction;
   private final Optimizer optimizer;

   private int iteration = 0;
   private DMatrixD1 optimumParameters = new DMatrixRMaj();
   private double optimumCost;

   private boolean verbose = true;

   public AugmentedLagrangeOptimizer(Optimizer optimizer, AugmentedLagrangeOptimizationProblem problem)
   {
      this.problem = problem;
      this.lagrangeCostFunction = problem.getAugmentedCostFunction();

      this.optimizer = optimizer;
   }

   public void setVerbose(boolean verbose)
   {
      this.verbose = verbose;
   }

   public DMatrixD1 solveOneIteration(DMatrixD1 initial)
   {


      return optimumParameters;
   }

   public DMatrixD1 optimize(int numLagrangeIterations, DMatrixD1 initialParam)
   {
      DMatrixD1 initial = new DMatrixRMaj(initialParam);
      iteration = 0;
      while (iteration < numLagrangeIterations)
      {
         optimizer.setCostFunction(lagrangeCostFunction);
         optimumParameters = optimizer.optimize(initial);
         optimumCost = optimizer.getOptimumCost();

         problem.updateLagrangeMultipliers(optimumParameters);
         initial.set(optimumParameters);

         if (verbose)
         {
            System.out.println("===== Lagrange Iteration: " + iteration + "==========");
            optimizer.printOutput();
         }
         iteration += 1;
      }

      return optimumParameters;
   }

   public double getOptimumCost()
   {
      return optimumCost;
   }
}
