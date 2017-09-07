package us.ihmc.robotics.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;

/**
* @author twan
*         Date: 8/9/13
*/
class ActiveSearchOptimizationSettings
{
   private final double epsilonConstraintActive;
   private final int maxIterations;
   private final LinearSolver<DenseMatrix64F> linearSolver;

   public ActiveSearchOptimizationSettings(double epsilonConstraintActive, int maxIterations, boolean useSVD)
   {
      this.epsilonConstraintActive = epsilonConstraintActive;
      this.maxIterations = maxIterations;
      this.linearSolver = LinearSolverFactory.pseudoInverse(useSVD);
   }

   public double getEpsilonConstraintActive()
   {
      return epsilonConstraintActive;
   }

   public int getMaxIterations()
   {
      return maxIterations;
   }

   public LinearSolver<DenseMatrix64F> getLinearSolver()
   {
      return linearSolver;
   }
}
