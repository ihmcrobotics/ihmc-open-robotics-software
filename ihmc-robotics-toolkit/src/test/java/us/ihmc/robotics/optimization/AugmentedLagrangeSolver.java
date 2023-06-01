package us.ihmc.robotics.optimization;

import org.ejml.data.DMatrixD1;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.robotics.optimization.constrainedOptimization.CostFunction;

public class AugmentedLagrangeSolver
{
   private final String name;

//   private final AugmentedLagrangeOptimizationProblem problem;
   private final CostFunction lagrangeCostFunction;
   private final Optimizer optimizer;

   private int iteration = 0;
   private DMatrixD1 initial = new DMatrixRMaj();
   private DMatrixD1 optimumParameters = new DMatrixRMaj();
   private double optimumCost;

   public AugmentedLagrangeSolver(Optimizer optimizer, CostFunction costFunction, DMatrixD1 initial, String name)
   {
//      this.problem = problem;
//      this.lagrangeCostFunction = problem.getAugmentedCostFunction();
      this.lagrangeCostFunction = costFunction;

      this.initial = new DMatrixRMaj(initial);
      this.name = name;

      this.optimizer = optimizer;
   }

   public DMatrixD1 solveOneIteration()
   {
      if (iteration > 0)
      {
         initial.set(optimumParameters);
      }

      optimizer.setCostFunction(lagrangeCostFunction);
      optimumParameters = optimizer.optimize(initial);
      optimumCost = optimizer.getOptimumCost();

//      problem.updateLagrangeMultipliers(optimumParameters);
      iteration += 1;

      return optimumParameters;
   }

//   public DMatrixD1 optimize(int numLagrangeIterations)
//   {
//      iteration = 0;
//      while (iteration < numLagrangeIterations)
//      {
//         solveOneIteration();
//      }
//      return optimumParameters;
//   }

   public double getOptimumCost()
   {
      return optimumCost;
   }

   public void printOutput()
   {
      for (int i = 0; i < optimumParameters.getNumElements(); i++)
         System.out.println(name + ": solution is " + optimumParameters.get(i));
      System.out.println("optimal cost is " + getOptimumCost());
   }
}
