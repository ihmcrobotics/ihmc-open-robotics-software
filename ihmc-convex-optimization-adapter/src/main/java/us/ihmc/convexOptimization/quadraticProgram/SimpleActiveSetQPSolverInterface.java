package us.ihmc.convexOptimization.quadraticProgram;

import org.ejml.data.DenseMatrix64F;

public interface SimpleActiveSetQPSolverInterface extends ActiveSetQPSolver
{
   public abstract void setUseWarmStart(boolean useWarmStart);

   public abstract void resetActiveConstraints();

   public abstract int solve(double[] solutionToPack, double[] lagrangeEqualityConstraintMultipliersToPack,
                             double[] lagrangeInequalityConstraintMultipliersToPack, double[] lagrangeLowerBoundMultipliersToPack,
                             double[] lagrangeUpperBoundMultipliersToPack);

   public abstract int solve(double[] solutionToPack, double[] lagrangeEqualityConstraintMultipliersToPack,
                             double[] lagrangeInequalityConstraintMultipliersToPack);

   public abstract int solve(DenseMatrix64F solutionToPack, DenseMatrix64F lagrangeEqualityConstraintMultipliersToPack,
                             DenseMatrix64F lagrangeInequalityConstraintMultipliersToPack, DenseMatrix64F lagrangeLowerBoundMultipliersToPack,
                             DenseMatrix64F lagrangeUpperBoundMultipliersToPack);

   public abstract int solve(DenseMatrix64F solutionToPack, DenseMatrix64F lagrangeEqualityConstraintMultipliersToPack,
                             DenseMatrix64F lagrangeInequalityConstraintMultipliersToPack);

   @Override
   public abstract int solve(DenseMatrix64F solutionToPack);

}