package us.ihmc.convexOptimization.quadraticProgram;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

public class SimpleActiveSetQPSolver extends AbstractActiveSetQPSolver
{
   // Uses the algorithm and naming convention found in MIT Paper
   // "An efficiently solvable quadratic program for stabilizing dynamic locomotion"
   // by Scott Kuindersma, Frank Permenter, and Russ Tedrake.
   DenseMatrix64F wInverse = new DenseMatrix64F(0);
   DenseMatrix64F gVector = new DenseMatrix64F(0);

   public SimpleActiveSetQPSolver()
   {
   }

   public double[] solve()
   {
      wInverse.reshape(quadraticCostGMatrix.numRows, quadraticCostGMatrix.numCols);
      wInverse.set(quadraticCostGMatrix);
      CommonOps.invert(wInverse); // can be sped up as w is block-diagonal

      gVector.reshape(quadraticCostFVector.numRows, quadraticCostFVector.numCols);
      gVector.set(quadraticCostFVector);

      DenseMatrix64F solutionMatrix = new DenseMatrix64F(numberOfVariablesToSolve, 1);

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
            DenseMatrix64F rMatrix = new DenseMatrix64F(rows, numberOfVariablesToSolve);

            if (linearEqualityConstraintsSize > 0)

               // setPartialMatrix(linearEqualityConstraintA, 0, 0, rMatrix);
               CommonOps.insert(linearEqualityConstraintA, rMatrix, 0, 0);
            if (activeSetSize > 0)
               setPartialMatrixForInequalityConstraints(linearInequalityConstraintA, linearInequalityActiveSet, linearEqualityConstraintsSize, 0, rMatrix);

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
               CommonOps.insert(linearEqualityConstraintB, eVector, 0, 0);

            // setPartialVector(linearEqualityConstraintsBVector, 0, eVector);
            if (activeSetSize > 0)
               setPartialVectorForInequalityConstraints(linearInequalityConstraintB, linearInequalityActiveSet, linearEqualityConstraintsSize, eVector);

            CommonOps.addEquals(rightSide, eVector);

            CommonOps.solve(leftSide, rightSide, alphaAndGamma);

            DenseMatrix64F temp2 = new DenseMatrix64F(numberOfVariablesToSolve, 1);
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
                  // For each element not in the active set, check to see
                  // if it should be in the active set if p_i^T z > f_i:

                  // DenseMatrix64F pVectorToCheck =
                  // createVector(linearInequalityConstraintPVectors[i]);
                  DenseMatrix64F pVectorToCheck = CommonOps.extract(linearInequalityConstraintA, i, i + 1, 0, linearInequalityConstraintA.numCols);
                  DenseMatrix64F temp3 = new DenseMatrix64F(1, 1);
                  CommonOps.mult(pVectorToCheck, solutionMatrix, temp3);

                  if (temp3.get(0, 0) > linearInequalityConstraintB.get(i, 0))
                  {
                     linearInequalityActiveSet[i] = true;
                     done = false;
                  }
               }
               else
               {
                  // For each element in the active set, check to see if
                  // it should be taken out of the active set if gamma_i <
                  // 0.0:

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

      double[] solution = new double[numberOfVariablesToSolve];

      for (int i = 0; i < numberOfVariablesToSolve; i++)
      {
         solution[i] = solutionMatrix.get(i, 0);
      }

      return solution;
   }

   protected static void setPartialMatrixForInequalityConstraints(DenseMatrix64F fromMatrix, boolean[] isActiveRowInMatrix, int startRow, int startColumn,
         DenseMatrix64F toMatrix)
   {
      int activeRow = 0;

      for (int i = 0; i < fromMatrix.numRows; i++)
      {
         if (isActiveRowInMatrix[i])
         {
            for (int j = 0; j < fromMatrix.numCols; j++)
            {
               toMatrix.set(startRow + activeRow, startColumn + j, fromMatrix.get(i, j));
            }

            activeRow++;
         }
      }

   }

   protected static void setPartialMatrixForInequalityConstraints(double[][] fromMatrix, boolean[] isActiveRowInMatrix, int startRow, int startColumn,
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

   protected void setPartialVectorForInequalityConstraints(DenseMatrix64F fromVector, boolean[] isActiveRowInVector, int startRow, DenseMatrix64F toVector)
   {
      int activeRow = 0;

      for (int i = 0; i < fromVector.numRows; i++)
      {
         if (isActiveRowInVector[i])
         {
            toVector.set(startRow + activeRow, 0, fromVector.get(i, 0));

            activeRow++;
         }
      }
   }

   protected void setPartialVectorForInequalityConstraints(double[] fromVector, boolean[] isActiveRowInVector, int startRow, DenseMatrix64F toVector)
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
}
