package us.ihmc.convexOptimization.quadraticProgram;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.tools.exceptions.NoConvergenceException;

public class SimpleEfficientActiveSetQPSolverWithInactiveVariables extends SimpleEfficientActiveSetQPSolver implements ActiveSetSolverWithInactiveVariablesInterface
{
   private final DenseMatrix64F originalQuadraticCostQMatrix = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F originalQuadraticCostQVector = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F originalLinearEqualityConstraintsAMatrix = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F originalLinearInequalityConstraintsCMatrixO = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F originalVariableLowerBounds = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F originalVariableUpperBounds = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F activeVariables = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F activeVariableSolution = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F computedObjectiveFunctionValue = new DenseMatrix64F(1, 1);

   private void setMatricesFromOriginal()
   {
      quadraticCostQMatrix.set(originalQuadraticCostQMatrix);
      quadraticCostQVector.set(originalQuadraticCostQVector);

      linearEqualityConstraintsAMatrix.set(originalLinearEqualityConstraintsAMatrix);
      linearInequalityConstraintsCMatrixO.set(originalLinearInequalityConstraintsCMatrixO);

      variableLowerBounds.set(originalVariableLowerBounds);
      variableUpperBounds.set(originalVariableUpperBounds);
   }

   private void removeInactiveVariables()
   {
      setMatricesFromOriginal();

      for (int variableIndex = activeVariables.getNumRows() - 1; variableIndex >= 0; variableIndex--)
      {
         if (activeVariables.get(variableIndex) == 1.0)
            continue;

         MatrixTools.removeRow(quadraticCostQMatrix, variableIndex);
         MatrixTools.removeColumn(quadraticCostQMatrix, variableIndex);

         MatrixTools.removeRow(quadraticCostQVector, variableIndex);

         if (linearEqualityConstraintsAMatrix.numRows > 0)
            MatrixTools.removeColumn(linearEqualityConstraintsAMatrix, variableIndex);
         if (linearInequalityConstraintsCMatrixO.numRows > 0)
            MatrixTools.removeColumn(linearInequalityConstraintsCMatrixO, variableIndex);

         if (variableLowerBounds.numRows > 0)
            MatrixTools.removeRow(variableLowerBounds, variableIndex);
         if (variableUpperBounds.numCols > 0)
            MatrixTools.removeRow(variableUpperBounds, variableIndex);
      }
   }

   private void copyActiveVariableSolutionToAllVariables(DenseMatrix64F solutionToPack, DenseMatrix64F activeVariableSolution)
   {
      if (MatrixTools.containsNaN(activeVariableSolution))
      {
         CommonOps.fill(solutionToPack, Double.NaN);
         return;
      }

      int activeVariableIndex = 0;
      for (int variableIndex = 0; variableIndex < solutionToPack.getNumRows(); variableIndex++)
      {
         if (activeVariables.get(variableIndex) != 1.0)
            continue;

         solutionToPack.set(variableIndex, 0, activeVariableSolution.get(activeVariableIndex, 0));
         activeVariableIndex++;
      }
   }

   @Override
   public void setLowerBounds(DenseMatrix64F variableLowerBounds)
   {
      if (variableLowerBounds.getNumRows() != originalQuadraticCostQMatrix.getNumRows())
         throw new RuntimeException("variableLowerBounds.getNumRows() != quadraticCostQMatrix.getNumRows()");

      this.originalVariableLowerBounds.set(variableLowerBounds);
   }

   @Override
   public void setUpperBounds(DenseMatrix64F variableUpperBounds)
   {
      if (variableUpperBounds.getNumRows() != originalQuadraticCostQMatrix.getNumRows())
         throw new RuntimeException("variableUpperBounds.getNumRows() != quadraticCostQMatrix.getNumRows()");

      this.originalVariableUpperBounds.set(variableUpperBounds);
   }

   @Override
   public void setQuadraticCostFunction(DenseMatrix64F costQuadraticMatrix, DenseMatrix64F costLinearVector, double quadraticCostScalar)
   {
      if (costLinearVector.getNumCols() != 1)
         throw new RuntimeException("costLinearVector.getNumCols() != 1");
      if (costQuadraticMatrix.getNumRows() != costLinearVector.getNumRows())
         throw new RuntimeException("costQuadraticMatrix.getNumRows() != costLinearVector.getNumRows()");
      if (costQuadraticMatrix.getNumRows() != costQuadraticMatrix.getNumCols())
         throw new RuntimeException("costQuadraticMatrix.getNumRows() != costQuadraticMatrix.getNumCols()");

      symmetricCostQuadraticMatrix.reshape(costQuadraticMatrix.getNumCols(), costQuadraticMatrix.getNumRows());
      CommonOps.transpose(costQuadraticMatrix, symmetricCostQuadraticMatrix);

      CommonOps.add(costQuadraticMatrix, symmetricCostQuadraticMatrix, symmetricCostQuadraticMatrix);
      CommonOps.scale(0.5, symmetricCostQuadraticMatrix);
      this.originalQuadraticCostQMatrix.set(symmetricCostQuadraticMatrix);
      this.originalQuadraticCostQVector.set(costLinearVector);
      this.quadraticCostScalar = quadraticCostScalar;

      setAllVariablesActive();
   }

   @Override
   public double getObjectiveCost(DenseMatrix64F x)
   {
      multQuad(x, originalQuadraticCostQMatrix, computedObjectiveFunctionValue);
      CommonOps.scale(0.5, computedObjectiveFunctionValue);
      CommonOps.multAddTransA(originalQuadraticCostQVector, x, computedObjectiveFunctionValue);
      return computedObjectiveFunctionValue.get(0, 0) + quadraticCostScalar;
   }

   @Override
   public void setLinearEqualityConstraints(DenseMatrix64F linearEqualityConstraintsAMatrix, DenseMatrix64F linearEqualityConstraintsBVector)
   {
      if (linearEqualityConstraintsBVector.getNumCols() != 1)
         throw new RuntimeException("linearEqualityConstraintsBVector.getNumCols() != 1");
      if (linearEqualityConstraintsAMatrix.getNumRows() != linearEqualityConstraintsBVector.getNumRows())
         throw new RuntimeException("linearEqualityConstraintsAMatrix.getNumRows() != linearEqualityConstraintsBVector.getNumRows()");
      if (linearEqualityConstraintsAMatrix.getNumCols() != originalQuadraticCostQMatrix.getNumCols())
         throw new RuntimeException("linearEqualityConstraintsAMatrix.getNumCols() != quadraticCostQMatrix.getNumCols()");

      this.linearEqualityConstraintsBVector.set(linearEqualityConstraintsBVector);
      this.originalLinearEqualityConstraintsAMatrix.set(linearEqualityConstraintsAMatrix);
   }

   @Override
   public void setLinearInequalityConstraints(DenseMatrix64F linearInequalityConstraintCMatrix, DenseMatrix64F linearInequalityConstraintDVector)
   {
      if (linearInequalityConstraintDVector.getNumCols() != 1)
         throw new RuntimeException("linearInequalityConstraintDVector.getNumCols() != 1");
      if (linearInequalityConstraintCMatrix.getNumRows() != linearInequalityConstraintDVector.getNumRows())
         throw new RuntimeException("linearInequalityConstraintCMatrix.getNumRows() != linearInequalityConstraintDVector.getNumRows()");
      if (linearInequalityConstraintCMatrix.getNumCols() != originalQuadraticCostQMatrix.getNumCols())
         throw new RuntimeException("linearInequalityConstraintCMatrix.getNumCols() != quadraticCostQMatrix.getNumCols()");

      this.linearInequalityConstraintsDVectorO.set(linearInequalityConstraintDVector);
      this.originalLinearInequalityConstraintsCMatrixO.set(linearInequalityConstraintCMatrix);
   }

   @Override
   public void setActiveVariables(DenseMatrix64F activeVariables)
   {
      if (activeVariables.getNumRows() != originalQuadraticCostQMatrix.getNumRows())
         throw new RuntimeException("activeVariables.getNumRows() != quadraticCostQMatrix.getNumRows()");

      this.activeVariables.set(activeVariables);
   }

   @Override
   public void setVariableActive(int variableIndex)
   {
      if (variableIndex >= activeVariables.getNumRows())
         throw new RuntimeException("variable index is outside the number of variables.");

      activeVariables.set(variableIndex, 1, 1.0);
   }

   @Override
   public void setVariableInactive(int variableIndex)
   {
      if (variableIndex >= activeVariables.getNumRows())
         throw new RuntimeException("variable index is outside the number of variables.");

      activeVariables.set(variableIndex, 1, 0.0);
   }

   @Override
   public void setAllVariablesActive()
   {
      activeVariables.reshape(originalQuadraticCostQMatrix.getNumRows(), 1);
      CommonOps.fill(activeVariables, 1.0);
   }

   @Override
   public void clear()
   {
      super.clear();

      activeVariables.reshape(0, 0);
      activeVariableSolution.reshape(0, 0);
   }

   @Override
   public int solve(double[] solutionToPack, double[] lagrangeEqualityConstraintMultipliersToPack, double[] lagrangeInequalityConstraintMultipliersToPack,
                    double[] lagrangeLowerBoundsConstraintMultipliersToPack, double[] lagrangeUpperBoundsConstraintMultipliersToPack)
   {
      setMatricesFromOriginal();

      return super.solve(solutionToPack, lagrangeEqualityConstraintMultipliersToPack, lagrangeInequalityConstraintMultipliersToPack,
                         lagrangeLowerBoundsConstraintMultipliersToPack, lagrangeUpperBoundsConstraintMultipliersToPack);
   }

   @Override
   public int solve(DenseMatrix64F solutionToPack, DenseMatrix64F lagrangeEqualityConstraintMultipliersToPack,
                    DenseMatrix64F lagrangeInequalityConstraintMultipliersToPack, DenseMatrix64F lagrangeLowerBoundConstraintMultipliersToPack,
                    DenseMatrix64F lagrangeUpperBoundConstraintMultipliersToPack)
   {
      removeInactiveVariables();

      activeVariableSolution.reshape(quadraticCostQMatrix.numRows, 1);

      int numberOfIterations = super.solve(activeVariableSolution, lagrangeEqualityConstraintMultipliersToPack, lagrangeInequalityConstraintMultipliersToPack,
                  lagrangeLowerBoundConstraintMultipliersToPack, lagrangeUpperBoundConstraintMultipliersToPack);


      copyActiveVariableSolutionToAllVariables(solutionToPack, activeVariableSolution);

      return numberOfIterations;
   }

}
