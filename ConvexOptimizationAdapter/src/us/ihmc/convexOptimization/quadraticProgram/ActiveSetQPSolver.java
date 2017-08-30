package us.ihmc.convexOptimization.quadraticProgram;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.tools.exceptions.NoConvergenceException;

public interface ActiveSetQPSolver
{
   void setConvergenceThreshold(double convergenceThreshold);

   void setMaxNumberOfIterations(int maxNumberOfIterations);

   void clear();

   void setLowerBounds(DenseMatrix64F variableLowerBounds);

   void setUpperBounds(DenseMatrix64F variableUpperBounds);

   default void setVariableBounds(DenseMatrix64F variableLowerBounds, DenseMatrix64F variableUpperBounds)
   {
      setLowerBounds(variableLowerBounds);
      setUpperBounds(variableUpperBounds);
   }

   default void setVariableBounds(double[] variableLowerBounds, double[] variableUpperBounds)
   {
      setVariableBounds(MatrixTools.createVector(variableLowerBounds), MatrixTools.createVector(variableUpperBounds));
   }

   default void setQuadraticCostFunction(double[][] quadraticCostFunctionQMatrix, double[] quadraticCostFunctionQVector, double quadraticCostScalar)
   {
      setQuadraticCostFunction(new DenseMatrix64F(quadraticCostFunctionQMatrix), MatrixTools.createVector(quadraticCostFunctionQVector), quadraticCostScalar);
   }

   void setQuadraticCostFunction(DenseMatrix64F costQuadraticMatrix, DenseMatrix64F costLinearVector, double quadraticCostScalar);

   double getObjectiveCost(DenseMatrix64F x);

   default void setLinearEqualityConstraints(double[][] linearEqualityConstraintsAMatrix, double[] linearEqualityConstraintsBVector)
   {
      setLinearEqualityConstraints(new DenseMatrix64F(linearEqualityConstraintsAMatrix), MatrixTools.createVector(linearEqualityConstraintsBVector));
   }

   void setLinearEqualityConstraints(DenseMatrix64F linearEqualityConstraintsAMatrix, DenseMatrix64F linearEqualityConstraintsBVector);

   default void setLinearInequalityConstraints(double[][] linearInequalityConstraintsCMatrix, double[] linearInqualityConstraintsDVector)
   {
      setLinearInequalityConstraints(new DenseMatrix64F(linearInequalityConstraintsCMatrix), MatrixTools.createVector(linearInqualityConstraintsDVector));
   }

   void setLinearInequalityConstraints(DenseMatrix64F linearInequalityConstraintCMatrix, DenseMatrix64F linearInequalityConstraintDVector);

   int solve(double[] solutionToPack) throws NoConvergenceException;

   int solve(DenseMatrix64F solutionToPack) throws NoConvergenceException;

}