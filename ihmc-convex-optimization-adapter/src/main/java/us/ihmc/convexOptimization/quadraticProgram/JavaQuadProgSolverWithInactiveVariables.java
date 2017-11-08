package us.ihmc.convexOptimization.quadraticProgram;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commons.MathTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;

public class JavaQuadProgSolverWithInactiveVariables extends JavaQuadProgSolver implements  ActiveSetQPSolverWithInactiveVariablesInterface
{
   private final DenseMatrix64F originalQuadraticCostQMatrix = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F originalQuadraticCostQVector = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F originalLinearEqualityConstraintsAMatrix = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F originalLinearEqualityConstraintsBVector = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F originalLinearInequalityConstraintsCMatrixO = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F originalLinearInequalityConstraintsDVectorO = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F originalLowerBoundsCMatrix = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F originalVariableLowerBounds = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F originalUpperBoundsCMatrix = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F originalVariableUpperBounds = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F activeVariables = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F activeVariableSolution = new DenseMatrix64F(0, 0);

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
   public void setLowerBounds(DenseMatrix64F variableLowerBounds)
   {
      int numberOfLowerBounds = variableLowerBounds.getNumRows();

      if (numberOfLowerBounds != originalQuadraticCostQMatrix.getNumRows())
         throw new RuntimeException("variableLowerBounds.getNumRows() != quadraticCostQMatrix.getNumRows()");

      originalLowerBoundsCMatrix.reshape(numberOfLowerBounds, numberOfLowerBounds);
      CommonOps.setIdentity(originalLowerBoundsCMatrix);

      this.originalVariableLowerBounds.set(variableLowerBounds);
      CommonOps.scale(-1.0, this.originalVariableLowerBounds);
   }

   @Override
   public void setUpperBounds(DenseMatrix64F variableUpperBounds)
   {
      int numberOfUpperBounds = variableUpperBounds.getNumRows();
      if (numberOfUpperBounds != originalQuadraticCostQMatrix.getNumRows())
         throw new RuntimeException("variableUpperBounds.getNumRows() != quadraticCostQMatrix.getNumRows()");

      originalUpperBoundsCMatrix.reshape(numberOfUpperBounds, numberOfUpperBounds);
      CommonOps.setIdentity(originalUpperBoundsCMatrix);
      CommonOps.scale(-1.0, originalUpperBoundsCMatrix);

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

      this.originalQuadraticCostQMatrix.set(costQuadraticMatrix);
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
      int numberOfEqualityConstraints = linearEqualityConstraintsBVector.getNumRows();

      if (linearEqualityConstraintsBVector.getNumCols() != 1)
         throw new RuntimeException("linearEqualityConstraintsBVector.getNumCols() != 1");
      if (linearEqualityConstraintsAMatrix.getNumRows() != linearEqualityConstraintsBVector.getNumRows())
         throw new RuntimeException("linearEqualityConstraintsAMatrix.getNumRows() != linearEqualityConstraintsBVector.getNumRows()");
      if (linearEqualityConstraintsAMatrix.getNumCols() != originalQuadraticCostQMatrix.getNumCols())
         throw new RuntimeException("linearEqualityConstraintsAMatrix.getNumCols() != quadraticCostQMatrix.getNumCols()");

      this.originalLinearEqualityConstraintsAMatrix.reshape(linearEqualityConstraintsAMatrix.getNumCols(), numberOfEqualityConstraints);
      CommonOps.transpose(linearEqualityConstraintsAMatrix, this.originalLinearEqualityConstraintsAMatrix);
      CommonOps.scale(-1.0, this.originalLinearEqualityConstraintsAMatrix);

      this.originalLinearEqualityConstraintsBVector.set(linearEqualityConstraintsBVector);
   }

   @Override
   public void setLinearInequalityConstraints(DenseMatrix64F linearInequalityConstraintCMatrix, DenseMatrix64F linearInequalityConstraintDVector)
   {
      int numberOfInequalityConstraints = linearInequalityConstraintDVector.getNumRows();

      if (linearInequalityConstraintDVector.getNumCols() != 1)
         throw new RuntimeException("linearInequalityConstraintDVector.getNumCols() != 1");
      if (linearInequalityConstraintCMatrix.getNumRows() != linearInequalityConstraintDVector.getNumRows())
         throw new RuntimeException("linearInequalityConstraintCMatrix.getNumRows() != linearInequalityConstraintDVector.getNumRows()");
      if (linearInequalityConstraintCMatrix.getNumCols() != originalQuadraticCostQMatrix.getNumCols())
         throw new RuntimeException("linearInequalityConstraintCMatrix.getNumCols() != quadraticCostQMatrix.getNumCols()");

      this.originalLinearInequalityConstraintsCMatrixO.reshape(linearInequalityConstraintCMatrix.getNumCols(), numberOfInequalityConstraints);
      CommonOps.transpose(linearInequalityConstraintCMatrix, this.originalLinearInequalityConstraintsCMatrixO);
      CommonOps.scale(-1.0, this.originalLinearInequalityConstraintsCMatrixO);

      this.originalLinearInequalityConstraintsDVectorO.set(linearInequalityConstraintDVector);
   }

   @Override
   public void clear()
   {
      super.clear();

      originalQuadraticCostQMatrix.reshape(0, 0);
      originalQuadraticCostQVector.reshape(0, 0);

      originalLinearEqualityConstraintsAMatrix.reshape(0, 0);
      originalLinearEqualityConstraintsBVector.reshape(0, 0);

      originalLinearInequalityConstraintsCMatrixO.reshape(0, 0);
      originalLinearInequalityConstraintsDVectorO.reshape(0, 0);

      originalLowerBoundsCMatrix.reshape(0, 0);
      originalVariableLowerBounds.reshape(0, 0);

      originalUpperBoundsCMatrix.reshape(0, 0);
      originalVariableUpperBounds.reshape(0, 0);

      activeVariables.reshape(0, 0);
      activeVariableSolution.reshape(0, 0);
   }

   @Override
   public int solve(double[] solutionToPack)
   {
      int numberOfEqualityConstraints = originalLinearEqualityConstraintsBVector.getNumRows();
      int numberOfInequalityConstraints = originalLinearInequalityConstraintsDVectorO.getNumRows();

      double[] lagrangeEqualityConstraintMultipliersToPack = new double[numberOfEqualityConstraints];
      double[] lagrangeInequalityConstraintMultipliersToPack = new double[numberOfInequalityConstraints];

      return solve(solutionToPack, lagrangeEqualityConstraintMultipliersToPack, lagrangeInequalityConstraintMultipliersToPack);
   }

   @Override
   public int solve(double[] solutionToPack, double[] lagrangeEqualityConstraintMultipliersToPack, double[] lagrangeInequalityConstraintMultipliersToPack)
   {
      int numberOfLowerBoundConstraints = originalVariableLowerBounds.getNumRows();
      int numberOfUpperBoundConstraints = originalVariableUpperBounds.getNumRows();

      double[] lagrangeLowerBoundsConstraintMultipliersToPack = new double[numberOfLowerBoundConstraints];
      double[] lagrangeUpperBoundsConstraintMultipliersToPack = new double[numberOfUpperBoundConstraints];

      return solve(solutionToPack, lagrangeEqualityConstraintMultipliersToPack, lagrangeInequalityConstraintMultipliersToPack,
                   lagrangeLowerBoundsConstraintMultipliersToPack, lagrangeUpperBoundsConstraintMultipliersToPack);
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

      solutionToPack.reshape(originalQuadraticCostQMatrix.numRows, 1);

      int numberOfIterations = super.solve(activeVariableSolution, lagrangeEqualityConstraintMultipliersToPack, lagrangeInequalityConstraintMultipliersToPack,
                                           lagrangeLowerBoundConstraintMultipliersToPack, lagrangeUpperBoundConstraintMultipliersToPack);


      copyActiveVariableSolutionToAllVariables(solutionToPack, activeVariableSolution);

      return numberOfIterations;
   }

   private void setMatricesFromOriginal()
   {
      quadraticCostQMatrix.set(originalQuadraticCostQMatrix);
      quadraticCostQVector.set(originalQuadraticCostQVector);

      linearEqualityConstraintsAMatrix.set(originalLinearEqualityConstraintsAMatrix);
      linearEqualityConstraintsBVector.set(originalLinearEqualityConstraintsBVector);

      linearInequalityConstraintsCMatrixO.set(originalLinearInequalityConstraintsCMatrixO);
      linearInequalityConstraintsDVectorO.set(originalLinearInequalityConstraintsDVectorO);

      lowerBoundsCMatrix.set(originalLowerBoundsCMatrix);
      variableLowerBounds.set(originalVariableLowerBounds);

      upperBoundsCMatrix.set(originalUpperBoundsCMatrix);
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

         if (linearEqualityConstraintsAMatrix.getNumElements() > 0)
            MatrixTools.removeRow(linearEqualityConstraintsAMatrix, variableIndex);
         if (linearInequalityConstraintsCMatrixO.getNumElements() > 0)
            MatrixTools.removeRow(linearInequalityConstraintsCMatrixO, variableIndex);

         if (variableLowerBounds.getNumElements() > 0)
         {
            MatrixTools.removeRow(variableLowerBounds, variableIndex);
            MatrixTools.removeRow(lowerBoundsCMatrix, variableIndex);
            MatrixTools.removeColumn(lowerBoundsCMatrix, variableIndex);
         }
         if (variableUpperBounds.getNumElements() > 0)
         {
            MatrixTools.removeRow(variableUpperBounds, variableIndex);
            MatrixTools.removeRow(upperBoundsCMatrix, variableIndex);
            MatrixTools.removeColumn(upperBoundsCMatrix, variableIndex);
         }
      }

      int numVars = quadraticCostQMatrix.getNumRows();
      if (linearEqualityConstraintsAMatrix.getNumElements() == 0)
         linearEqualityConstraintsAMatrix.reshape(numVars, 0);
      if (linearInequalityConstraintsCMatrixO.getNumElements() == 0)
         linearInequalityConstraintsCMatrixO.reshape(numVars, 0);

      removeZeroColumnsFromConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);
      removeZeroColumnsFromConstraints(linearInequalityConstraintsCMatrixO, linearInequalityConstraintsDVectorO);
   }

   private static void removeZeroColumnsFromConstraints(DenseMatrix64F matrix, DenseMatrix64F vector)
   {
      for (int rowIndex = vector.numRows - 1; rowIndex >= 0; rowIndex--)
      {
         double sumOfRowElements = 0.0;

         for (int columnIndex = 0; columnIndex < matrix.getNumRows(); columnIndex++)
         {
            sumOfRowElements += Math.abs(matrix.get(columnIndex, rowIndex));
         }

         boolean isZeroColumn = MathTools.epsilonEquals(sumOfRowElements, 0.0, 1e-12);
         if (isZeroColumn)
         {
            MatrixTools.removeColumn(matrix, rowIndex);
            MatrixTools.removeRow(vector, rowIndex);
         }
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
}
