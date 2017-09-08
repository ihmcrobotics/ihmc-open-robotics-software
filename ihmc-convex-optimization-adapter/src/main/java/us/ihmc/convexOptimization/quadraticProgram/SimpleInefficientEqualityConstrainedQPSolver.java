package us.ihmc.convexOptimization.quadraticProgram;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.linearAlgebra.MatrixTools;

public class SimpleInefficientEqualityConstrainedQPSolver
{
   private final DenseMatrix64F quadraticCostQMatrix = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F quadraticCostQVector = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F linearEqualityConstraintsAMatrix = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F linearEqualityConstraintsAMatrixTranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F linearEqualityConstraintsBVector = new DenseMatrix64F(0, 0);

   private double quadraticCostScalar;

   private int numberOfVariablesToSolve;

   public SimpleInefficientEqualityConstrainedQPSolver()
   {
      numberOfVariablesToSolve = -1; //indicate unknown size
   }

   public void clear()
   {
      numberOfVariablesToSolve = -1;

      linearEqualityConstraintsAMatrix.reshape(0, 0);
      linearEqualityConstraintsAMatrixTranspose.reshape(0, 0);
      linearEqualityConstraintsBVector.reshape(0, 0);
      quadraticCostQMatrix.reshape(0, 0);
      quadraticCostQVector.reshape(0, 0);
   }

   //   public int getNumEqualityConstraints()
   //   {
   //      return linearEqualityConstraintAMatrix.getNumRows();
   //   }

   public void setQuadraticCostFunction(double[][] quadraticCostFunctionWMatrix, double[] quadraticCostFunctionGVector, double quadraticCostScalar)
   {
      assertCorrectSize(quadraticCostFunctionWMatrix);
      assertCorrectSize(quadraticCostFunctionGVector);
      setQuadraticCostFunction(new DenseMatrix64F(quadraticCostFunctionWMatrix), MatrixTools.createVector(quadraticCostFunctionGVector), quadraticCostScalar);
   }

   public void setQuadraticCostFunction(DenseMatrix64F quadraticCostQMatrix, DenseMatrix64F quadraticCostQVector, double quadraticCostScalar)
   {
      setAndAssertCorrectNumberOfVariablesToSolve(quadraticCostQMatrix.numCols);
      setAndAssertCorrectNumberOfVariablesToSolve(quadraticCostQMatrix.numRows);
      setAndAssertCorrectNumberOfVariablesToSolve(quadraticCostQVector.numRows);
      DenseMatrix64F symCostQuadraticMatrix = new DenseMatrix64F(quadraticCostQMatrix);
      CommonOps.transpose(symCostQuadraticMatrix);
      CommonOps.add(quadraticCostQMatrix, symCostQuadraticMatrix, symCostQuadraticMatrix);
      CommonOps.scale(0.5, symCostQuadraticMatrix);
      this.quadraticCostQMatrix.set(symCostQuadraticMatrix);
      this.quadraticCostQVector.set(quadraticCostQVector);
      this.quadraticCostScalar = quadraticCostScalar;
   }

   public void setLinearEqualityConstraints(double[][] linearEqualityConstraintsAMatrix, double[] linearEqualityConstraintsBVector)
   {
      assertCorrectColumnSize(linearEqualityConstraintsAMatrix);
      if (linearEqualityConstraintsAMatrix.length != linearEqualityConstraintsBVector.length)
         throw new RuntimeException();
      setLinearEqualityConstraints(new DenseMatrix64F(linearEqualityConstraintsAMatrix), MatrixTools.createVector(linearEqualityConstraintsBVector));
   }

   public void setLinearEqualityConstraints(DenseMatrix64F linearEqualityConstraintsAMatrix, DenseMatrix64F linearEqualityConstraintsBVector)
   {
      setAndAssertCorrectNumberOfVariablesToSolve(linearEqualityConstraintsAMatrix.numCols);
      this.linearEqualityConstraintsBVector.set(linearEqualityConstraintsBVector);
      this.linearEqualityConstraintsAMatrix.set(linearEqualityConstraintsAMatrix);
      this.linearEqualityConstraintsAMatrixTranspose.set(CommonOps.transpose(linearEqualityConstraintsAMatrix, null));
   }

   //   protected int getLinearEqualityConstraintsSize()
   //   {
   //      if (linearEqualityConstraintAMatrix == null)
   //         return 0;
   //
   //      return linearEqualityConstraintAMatrix.numRows;
   //   }

   private void assertCorrectSize(double[][] matrix)
   {
      if (numberOfVariablesToSolve == -1)
      {
         numberOfVariablesToSolve = matrix.length;
      }

      if (matrix.length != numberOfVariablesToSolve)
         throw new RuntimeException("matrix.length = " + matrix.length + " != numberOfVariablesToSolve = " + numberOfVariablesToSolve);
      if (matrix[0].length != numberOfVariablesToSolve)
         throw new RuntimeException("matrix[0].length = " + matrix[0].length + " != numberOfVariablesToSolve = " + numberOfVariablesToSolve);
   }

   private void assertCorrectColumnSize(double[][] matrix)
   {
      if (numberOfVariablesToSolve == -1)
      {
         numberOfVariablesToSolve = matrix[0].length;
      }

      if (matrix[0].length != numberOfVariablesToSolve)
         throw new RuntimeException("matrix[0].length = " + matrix[0].length + " != numberOfVariablesToSolve = " + numberOfVariablesToSolve);
   }

   protected void setAndAssertCorrectNumberOfVariablesToSolve(int n)
   {
      if (numberOfVariablesToSolve == -1)
      {
         numberOfVariablesToSolve = n;
      }

      if (n != numberOfVariablesToSolve)
         throw new RuntimeException("incorrect NumberOfVariables size");
   }

   private void assertCorrectSize(double[] vector)
   {
      if (numberOfVariablesToSolve == -1)
      {
         numberOfVariablesToSolve = vector.length;
      }

      if (vector.length != numberOfVariablesToSolve)
         throw new RuntimeException("vector.length = " + vector.length + " != numberOfVariablesToSolve = " + numberOfVariablesToSolve);
   }

   public void solve(double[] xSolutionToPack, double[] lagrangeMultipliersToPack)
   {
      int numberOfVariables = quadraticCostQMatrix.getNumCols();
      int numberOfEqualityConstraints = linearEqualityConstraintsAMatrix.getNumRows();

      if (xSolutionToPack.length != numberOfVariables)
         throw new RuntimeException("xSolutionToPack.length != numberOfVariables");
      if (lagrangeMultipliersToPack.length != numberOfEqualityConstraints)
         throw new RuntimeException("lagrangeMultipliersToPack.length != numberOfEqualityConstraints");

      DenseMatrix64F solution = new DenseMatrix64F(numberOfVariables, 1);
      DenseMatrix64F lagrangeMultipliers = new DenseMatrix64F(numberOfEqualityConstraints, 1);

      solve(solution, lagrangeMultipliers);

      double[] solutionData = solution.getData();

      for (int i = 0; i < numberOfVariables; i++)
      {
         xSolutionToPack[i] = solutionData[i];
      }

      double[] lagrangeMultipliersData = lagrangeMultipliers.getData();

      for (int i = 0; i < numberOfEqualityConstraints; i++)
      {
         lagrangeMultipliersToPack[i] = lagrangeMultipliersData[i];
      }
   }

   public void solve(DenseMatrix64F xSolutionToPack, DenseMatrix64F lagrangeMultipliersToPack)
   {
      int numberOfVariables = quadraticCostQMatrix.getNumCols();
      if (numberOfVariables != quadraticCostQMatrix.getNumRows())
         throw new RuntimeException("numCols != numRows");

      int numberOfEqualityConstraints = linearEqualityConstraintsAMatrix.getNumRows();
      if (numberOfEqualityConstraints > 0)
      {
         if (linearEqualityConstraintsAMatrix.getNumCols() != numberOfVariables)
            throw new RuntimeException("linearEqualityConstraintA.getNumCols() != numberOfVariables");
      }

      if (quadraticCostQVector.getNumRows() != numberOfVariables)
         throw new RuntimeException("quadraticCostQVector.getNumRows() != numRows");
      if (quadraticCostQVector.getNumCols() != 1)
         throw new RuntimeException("quadraticCostQVector.getNumCols() != 1");

      DenseMatrix64F negativeQuadraticCostQVector = new DenseMatrix64F(quadraticCostQVector);
      CommonOps.scale(-1.0, negativeQuadraticCostQVector);

      if (numberOfEqualityConstraints == 0)
      {
         xSolutionToPack.reshape(numberOfVariables, 1);
         CommonOps.solve(quadraticCostQMatrix, negativeQuadraticCostQVector, xSolutionToPack);
         return;
      }

      CommonOps.transpose(linearEqualityConstraintsAMatrix, linearEqualityConstraintsAMatrixTranspose);
      DenseMatrix64F bigMatrix = new DenseMatrix64F(numberOfVariables + numberOfEqualityConstraints, numberOfVariables + numberOfEqualityConstraints);
      DenseMatrix64F bigVector = new DenseMatrix64F(numberOfVariables + numberOfEqualityConstraints, 1);

      CommonOps.insert(quadraticCostQMatrix, bigMatrix, 0, 0);
      CommonOps.insert(linearEqualityConstraintsAMatrix, bigMatrix, numberOfVariables, 0);
      CommonOps.insert(linearEqualityConstraintsAMatrixTranspose, bigMatrix, 0, numberOfVariables);

      CommonOps.insert(negativeQuadraticCostQVector, bigVector, 0, 0);
      CommonOps.insert(linearEqualityConstraintsBVector, bigVector, numberOfVariables, 0);

      DenseMatrix64F xAndLagrangeMultiplierSolution = new DenseMatrix64F(numberOfVariables + numberOfEqualityConstraints, 1);
      CommonOps.solve(bigMatrix, bigVector, xAndLagrangeMultiplierSolution);

      for (int i = 0; i < numberOfVariables; i++)
      {
         xSolutionToPack.set(i, 0, xAndLagrangeMultiplierSolution.get(i, 0));
      }

      for (int i = 0; i < numberOfEqualityConstraints; i++)
      {
         lagrangeMultipliersToPack.set(i, 0, xAndLagrangeMultiplierSolution.get(numberOfVariables + i, 0));
      }
   }

   //   protected static void setPartialMatrix(double[][] fromMatrix, int startRow, int startColumn, DenseMatrix64F toMatrix)
   //   {
   //      for (int i = 0; i < fromMatrix.length; i++)
   //      {
   //         for (int j = 0; j < fromMatrix[0].length; j++)
   //         {
   //            toMatrix.set(startRow + i, startColumn + j, fromMatrix[i][j]);
   //         }
   //      }
   //   }
   //
   //   protected static void setPartialVector(double[] fromVector, int startRow, DenseMatrix64F toVector)
   //   {
   //      for (int i = 0; i < fromVector.length; i++)
   //      {
   //         toVector.set(startRow + i, 0, fromVector[i]);
   //      }
   //   }

   private final DenseMatrix64F computedObjectiveFunctionValue = new DenseMatrix64F(1, 1);

   public double getObjectiveCost(DenseMatrix64F x)
   {
      MatrixTools.multQuad(x, quadraticCostQMatrix, computedObjectiveFunctionValue);
      CommonOps.scale(0.5, computedObjectiveFunctionValue);
      CommonOps.multAddTransA(quadraticCostQVector, x, computedObjectiveFunctionValue);
      return computedObjectiveFunctionValue.get(0, 0) + quadraticCostScalar;
   }

   public void displayProblem()
   {
      //      setZeroSizeMatrixForNullFields();
      System.out.println("----------------------------------------------------------------------------------------------------");
      System.out.println("equalityA:" + linearEqualityConstraintsAMatrix);
      System.out.println("equalityB:" + linearEqualityConstraintsBVector);
      System.out.println("costQuadQ:" + quadraticCostQMatrix);
      System.out.println("costLinearF:" + quadraticCostQVector);
      System.out.println("costLinearScalar:" + quadraticCostScalar);
      System.out.println("----------------------------------------------------------------------------------------------------");
   }

   //   boolean isNullFieldSet = false;
   //
   //   public void setZeroSizeMatrixForNullFields()
   //   {
   //      if (isNullFieldSet)
   //         return;
   //      assert (numberOfVariablesToSolve > 0);
   //      if (linearEqualityConstraintA == null)
   //      {
   //         linearEqualityConstraintA = new DenseMatrix64F(0, numberOfVariablesToSolve);
   //         linearEqualityConstraintATranspose = new DenseMatrix64F(numberOfVariablesToSolve, 0);
   //         linearEqualityConstraintB = new DenseMatrix64F(0, 1);
   //      }
   //
   //      if (quadraticCostGMatrix == null)
   //      {
   //         quadraticCostGMatrix = new DenseMatrix64F(numberOfVariablesToSolve, numberOfVariablesToSolve);
   //      }
   //      if (quadraticCostFVector == null)
   //      {
   //         quadraticCostFVector = new DenseMatrix64F(numberOfVariablesToSolve, 1);
   //      }
   //   }

}
