package us.ihmc.robotics.optimization;

import org.ejml.data.DMatrixD1;
import us.ihmc.robotics.optimization.constrainedOptimization.CostFunction;

/**
 * Interface for a R^n -> R optimizer
 */
public interface Optimizer
{
   public void setCostFunction(CostFunction costFunction);

   /**
    * Optimizes over a single iteration
    * @return the current optimum at the end of the iteration
    */
   public DMatrixD1 stepOneIteration();

   /**
    * Optimizes until end conditions are reached
    * @return the optimum that was found
    */
   public DMatrixD1 optimize(DMatrixD1 initial);

   public DMatrixD1 getOptimalParameters();

   public double getOptimumCost();

   default public void printOutput()
   {
      System.out.println("solution is: ");
      for (int i = 0; i < getOptimalParameters().getNumElements(); i++)
      {
         System.out.println(getOptimalParameters().get(i) + ",");
      }
      System.out.println("optimal cost is " + getOptimumCost());
   }
}
