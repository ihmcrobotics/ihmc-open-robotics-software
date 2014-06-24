package us.ihmc.convexOptimization.quadraticProgram;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

public class SimpleActiveSetQPSolver
{
   // Uses the algorithm and naming convention found in MIT Paper
   // "An efficiently solvable quadratic program for stabilizing dynamic locomotion"
   // by Scott Kuindersma, Frank Permenter, and Russ Tedrake.

   private int numberOfVariablesToSolve;

   private double[][] quadraticCostFunctionWMatrix;
   private double[] quadraticCostFunctionGVector;
   private double[][] linearEqualityConstraintsAMatrix;
   private double[] linearEqualityConstraintsBVector;
   private double[][] linearInequalityConstraintPVectors;
   private double[] linearInequalityConstraintFs;

   private boolean[] linearInequalityActiveSet;

   public SimpleActiveSetQPSolver()
   {
      numberOfVariablesToSolve = -1;
   }

   public void setQuadraticCostFunction(double[][] quadraticCostFunctionWMatrix, double[] quadraticCostFunctionGVector, double quadraticCostFunctionR)
   {
      assertCorrectSize(quadraticCostFunctionWMatrix);
      assertCorrectSize(quadraticCostFunctionGVector);

      this.quadraticCostFunctionWMatrix = quadraticCostFunctionWMatrix;
      this.quadraticCostFunctionGVector = quadraticCostFunctionGVector;
   }

   public void setLinearEqualityConstraintsAMatrix(double[][] linearEqualityConstraintsAMatrix)
   {
      assertCorrectColumnSize(linearEqualityConstraintsAMatrix);

      this.linearEqualityConstraintsAMatrix = linearEqualityConstraintsAMatrix;
   }

   public void setLinearEqualityConstraintsBVector(double[] linearEqualityConstraintsBVector)
   {
      if (linearEqualityConstraintsAMatrix != null)
      {
         if (linearEqualityConstraintsAMatrix.length != linearEqualityConstraintsBVector.length)
            throw new RuntimeException();
      }

      this.linearEqualityConstraintsBVector = linearEqualityConstraintsBVector;
   }

   public void setLinearInequalityConstraints(double[][] linearInequalityConstraintPVectors, double[] linearInequalityConstraintFs)
   {
      assertCorrectColumnSize(linearInequalityConstraintPVectors);
      if (linearInequalityConstraintPVectors.length != linearInequalityConstraintFs.length)
         throw new RuntimeException();

      this.linearInequalityConstraintPVectors = linearInequalityConstraintPVectors;
      this.linearInequalityConstraintFs = linearInequalityConstraintFs;

      this.linearInequalityActiveSet = new boolean[linearInequalityConstraintPVectors.length];

      for (int i = 0; i < linearInequalityActiveSet.length; i++)
      {
         this.linearInequalityActiveSet[i] = false;
      }
   }
   
   public void setLinearInequalityActiveSet(boolean[] linearInequalityActiveSet)
   {
      if (this.linearInequalityActiveSet.length != linearInequalityActiveSet.length) throw new RuntimeException();
      
      for (int i=0; i<linearInequalityActiveSet.length; i++)
      {
         this.linearInequalityActiveSet[i] = linearInequalityActiveSet[i];
      }
   }

   private int getLinearEqualityConstraintsSize()
   {
      if (linearEqualityConstraintsAMatrix == null)
         return 0;

      return linearEqualityConstraintsAMatrix.length;
   }

   private int getActiveSetSize()
   {
      if (linearInequalityActiveSet == null)
         return 0;

      int ret = 0;

      for (int i = 0; i < linearInequalityActiveSet.length; i++)
      {
         if (linearInequalityActiveSet[i])
            ret++;
      }

      return ret;
   }

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


   private void assertCorrectSize(double[] vector)
   {
      if (numberOfVariablesToSolve == -1)
      {
         numberOfVariablesToSolve = vector.length;
      }

      if (vector.length != numberOfVariablesToSolve)
         throw new RuntimeException("vector.length = " + vector.length + " != numberOfVariablesToSolve = " + numberOfVariablesToSolve);
   }


   public double[] solve()
   {
      DenseMatrix64F wInverse = new DenseMatrix64F(quadraticCostFunctionWMatrix);
      DenseMatrix64F gVector = createVector(quadraticCostFunctionGVector);
      CommonOps.invert(wInverse);

      int numberOfVariables = quadraticCostFunctionWMatrix.length;
      DenseMatrix64F solutionMatrix = new DenseMatrix64F(numberOfVariables, 1);

      int linearEqualityConstraintsSize = getLinearEqualityConstraintsSize();

      int maxIterations = 100;
      int iterations = 0;
      boolean done = false;


      while (!done)
      {
         int activeSetSize = getActiveSetSize();
         int rows = linearEqualityConstraintsSize + activeSetSize;
         DenseMatrix64F alphaAndGamma = new DenseMatrix64F(rows, 1);


         if ((linearEqualityConstraintsSize > 0) || (activeSetSize > 0))
         {
            DenseMatrix64F rMatrix = new DenseMatrix64F(rows, numberOfVariables);

            if (linearEqualityConstraintsSize > 0)
               setPartialMatrix(linearEqualityConstraintsAMatrix, 0, 0, rMatrix);
            if (activeSetSize > 0)
               setPartialMatrixForInequalityConstraints(linearInequalityConstraintPVectors, linearInequalityActiveSet, linearEqualityConstraintsSize, 0,
                       rMatrix);

            DenseMatrix64F rTransposeMatrix = new DenseMatrix64F(rMatrix);
            CommonOps.transpose(rTransposeMatrix);

            DenseMatrix64F temp1 = new DenseMatrix64F(rMatrix.getNumRows(), wInverse.getNumCols());
            DenseMatrix64F leftSide = new DenseMatrix64F(rMatrix.getNumRows(), rMatrix.getNumRows());

            CommonOps.mult(rMatrix, wInverse, temp1);
            CommonOps.mult(temp1, rTransposeMatrix, leftSide);
            CommonOps.scale(-1.0, leftSide);

            DenseMatrix64F rightSide = new DenseMatrix64F(rMatrix.getNumRows(), 1);
            CommonOps.mult(temp1, gVector, rightSide);

            DenseMatrix64F eVector = new DenseMatrix64F(rows, 1);
            if (linearEqualityConstraintsSize > 0)
               setPartialVector(linearEqualityConstraintsBVector, 0, eVector);
            if (activeSetSize > 0)
               setPartialVectorForInequalityConstraints(linearInequalityConstraintFs, linearInequalityActiveSet, linearEqualityConstraintsSize, eVector);

            CommonOps.addEquals(rightSide, eVector);

            CommonOps.solve(leftSide, rightSide, alphaAndGamma);

            DenseMatrix64F temp2 = new DenseMatrix64F(numberOfVariables, 1);
            CommonOps.mult(rTransposeMatrix, alphaAndGamma, temp2);
            CommonOps.addEquals(temp2, gVector);

            CommonOps.mult(wInverse, temp2, solutionMatrix);
            CommonOps.scale(-1.0, solutionMatrix);
         }

         else
         {
            CommonOps.mult(wInverse, gVector, solutionMatrix);
            CommonOps.scale(-1.0, solutionMatrix);
         }

         iterations++;

         if (iterations > maxIterations)
         {
            done = true;
         }
         else if (linearInequalityActiveSet != null)
         {
            done = true;
            
            for (int i = 0; i < linearInequalityActiveSet.length; i++)
            {
               if (linearInequalityActiveSet[i] == false)
               {
                  // For each element not in the active set, check to see if it should be in the active set if p_i^T z > f_i:

                  DenseMatrix64F pVectorToCheck = createVector(linearInequalityConstraintPVectors[i]);
                  DenseMatrix64F temp3 = new DenseMatrix64F(1, 1);
                  CommonOps.multTransA(pVectorToCheck, solutionMatrix, temp3);
                  if (temp3.get(0,0) > linearInequalityConstraintFs[i])
                  {
                     linearInequalityActiveSet[i] = true;
                     done = false;
                  }
               }
               else
               {
                  // For each element in the active set, check to see if it should be taken out of the active set if gamma_i < 0.0:

                  double gamma = alphaAndGamma.get(linearEqualityConstraintsSize + i, 0);
                  if (gamma < 0.0) 
                  {
                     linearInequalityActiveSet[i] = false;
                     done = false;
                  }
               }
            }
         }
      }

      double[] solution = new double[numberOfVariables];

      for (int i = 0; i < numberOfVariables; i++)
      {
         solution[i] = solutionMatrix.get(i, 0);
      }

      return solution;
   }



   private void setPartialMatrix(double[][] fromMatrix, int startRow, int startColumn, DenseMatrix64F toMatrix)
   {
      for (int i = 0; i < fromMatrix.length; i++)
      {
         for (int j = 0; j < fromMatrix[0].length; j++)
         {
            toMatrix.set(startRow + i, startColumn + j, fromMatrix[i][j]);
         }
      }
   }


   private void setPartialVector(double[] fromVector, int startRow, DenseMatrix64F toVector)
   {
      for (int i = 0; i < fromVector.length; i++)
      {
         toVector.set(startRow + i, 0, fromVector[i]);
      }
   }

   private void setPartialMatrixForInequalityConstraints(double[][] fromMatrix, boolean[] isActiveRowInMatrix, int startRow, int startColumn,
           DenseMatrix64F toMatrix)
   {
      int activeRow = 0;

      for (int i = 0; i < fromMatrix.length; i++)
      {
         if (isActiveRowInMatrix[i])
         {
            for (int j = 0; j < fromMatrix[0].length; j++)
            {
               toMatrix.set(startRow + activeRow, startColumn + j, fromMatrix[i][j]);
            }

            activeRow++;
         }
      }
   }

   private void setPartialVectorForInequalityConstraints(double[] fromVector, boolean[] isActiveRowInVector, int startRow, DenseMatrix64F toVector)
   {
      int activeRow = 0;

      for (int i = 0; i < fromVector.length; i++)
      {
         if (isActiveRowInVector[i])
         {
            toVector.set(startRow + activeRow, 0, fromVector[i]);

            activeRow++;
         }
      }
   }


   private void setVector(DenseMatrix64F matrixToSet, double[] vectorElements)
   {
      for (int i = 0; i < vectorElements.length; i++)
      {
         matrixToSet.set(i, 0, vectorElements[i]);
      }
   }

   private DenseMatrix64F createVector(double[] vectorElements)
   {
      DenseMatrix64F ret = new DenseMatrix64F(vectorElements.length, 1);
      setVector(ret, vectorElements);

      return ret;
   }

   public void dispose()
   {
   }

}
