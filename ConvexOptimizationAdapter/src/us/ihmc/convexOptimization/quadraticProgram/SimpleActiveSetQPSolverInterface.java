package us.ihmc.convexOptimization.quadraticProgram;

import org.ejml.data.DenseMatrix64F;

public interface SimpleActiveSetQPSolverInterface
{
   public abstract void setVariableBounds(double[] variableLowerBounds, double[] variableUpperBounds);

   public abstract void setVariableBounds(DenseMatrix64F variableLowerBounds, DenseMatrix64F variableUpperBounds);

   public abstract void setMaxNumberOfIterations(int maxNumberOfIterations);

   public abstract void clear();

   public abstract void setQuadraticCostFunction(double[][] quadraticCostFunctionWMatrix, double[] quadraticCostFunctionGVector, double quadraticCostScalar);

   public abstract void setQuadraticCostFunction(DenseMatrix64F costQuadraticMatrix, DenseMatrix64F costLinearVector, double quadraticCostScalar);

   public abstract void setQuadraticCostFunction(DenseMatrix64F costQuadraticMatrix, DenseMatrix64F costLinearVector, double quadraticCostScalar,
         boolean quadraticCostMatrixIsDiagonal);

   public abstract double getObjectiveCost(DenseMatrix64F solutionMatrix);

   public abstract void setLinearEqualityConstraints(double[][] linearEqualityConstraintsAMatrix, double[] linearEqualityConstraintsBVector);

   public abstract void setLinearEqualityConstraints(DenseMatrix64F linearEqualityConstraintsAMatrix, DenseMatrix64F linearEqualityConstraintsBVector);

   public abstract void setLinearInequalityConstraints(double[][] linearInequalityConstraintsCMatrix, double[] linearInqualityConstraintsDVector);

   public abstract void setLinearInequalityConstraints(DenseMatrix64F linearInequalityConstraintCMatrix, DenseMatrix64F linearInequalityConstraintDVector);

   public abstract void setUseWarmStart(boolean useWarmStart);

   public abstract void resetActiveConstraints();

   public abstract int solve(double[] solutionToPack, double[] lagrangeEqualityConstraintMultipliersToPack, double[] lagrangeInequalityConstraintMultipliersToPack, 
         double[] lagrangeLowerBoundMultipliersToPack, double[] lagrangeUpperBoundMultipliersToPack);
   
   public abstract int solve(double[] solutionToPack, double[] lagrangeEqualityConstraintMultipliersToPack, double[] lagrangeInequalityConstraintMultipliersToPack);

   public abstract int solve(double[] solutionToPack);

   public abstract int solve(DenseMatrix64F solutionToPack, DenseMatrix64F lagrangeEqualityConstraintMultipliersToPack, DenseMatrix64F lagrangeInequalityConstraintMultipliersToPack, 
         DenseMatrix64F lagrangeLowerBoundMultipliersToPack, DenseMatrix64F lagrangeUpperBoundMultipliersToPack);
   
   public abstract int solve(DenseMatrix64F solutionToPack, DenseMatrix64F lagrangeEqualityConstraintMultipliersToPack, DenseMatrix64F lagrangeInequalityConstraintMultipliersToPack);

   public abstract int solve(DenseMatrix64F solutionToPack);

}