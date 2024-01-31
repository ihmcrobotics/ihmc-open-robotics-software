package us.ihmc.robotics.optimization;

import org.ejml.data.DMatrixD1;

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
   DMatrixD1 optimize(DMatrixD1 initial);

   DMatrixD1 getOptimalParameters();

   double getOptimumCost();

   default void printResults()
   {
      System.out.println("Solution x*: ");
      for (int i = 0; i < getOptimalParameters().getNumElements(); i++)
      {
         System.out.println("\t" + getOptimalParameters().get(i) + ",");
      }
      System.out.println("Local Optimum f(x*): " + getOptimumCost());
   }

}
