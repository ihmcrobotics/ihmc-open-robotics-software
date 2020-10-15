package us.ihmc.robotics.optimization;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;

/**
* @author twan
*         Date: 8/9/13
*/
class ActiveSearchOptimizationSettings
{
   private final double epsilonConstraintActive;
   private final int maxIterations;
   private final LinearSolverDense<DMatrixRMaj> linearSolver;

   public ActiveSearchOptimizationSettings(double epsilonConstraintActive, int maxIterations, boolean useSVD)
   {
      this.epsilonConstraintActive = epsilonConstraintActive;
      this.maxIterations = maxIterations;
      this.linearSolver = LinearSolverFactory_DDRM.pseudoInverse(useSVD);
   }

   public double getEpsilonConstraintActive()
   {
      return epsilonConstraintActive;
   }

   public int getMaxIterations()
   {
      return maxIterations;
   }

   public LinearSolverDense<DMatrixRMaj> getLinearSolver()
   {
      return linearSolver;
   }
}
