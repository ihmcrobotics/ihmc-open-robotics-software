package us.ihmc.convexOptimization.quadraticProgram;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.utilities.math.MatrixTools;

public class SimpleActiveSetQPSolver extends AbstractActiveSetQPSolver
{
   // Uses the algorithm and naming convention found in MIT Paper
   // "An efficiently solvable quadratic program for stabilizing dynamic locomotion"
   // by Scott Kuindersma, Frank Permenter, and Russ Tedrake.
   DenseMatrix64F wInverse = new DenseMatrix64F(0);
   DenseMatrix64F wInverseG = new DenseMatrix64F(0);
   DenseMatrix64F gVector = new DenseMatrix64F(0);
   DenseMatrix64F alphaAndGamma = new DenseMatrix64F(0);
   DenseMatrix64F solutionVector = new DenseMatrix64F(0);
   DenseMatrix64F rMatrix = new DenseMatrix64F(0);

   DenseMatrix64F wInverseRTranspose = new DenseMatrix64F(0);
   DenseMatrix64F leftSide = new DenseMatrix64F(0);
   DenseMatrix64F rightSide = new DenseMatrix64F(0);
   DenseMatrix64F eVector = new DenseMatrix64F(0);

   public SimpleActiveSetQPSolver()
   {
   }

   public double[] solve()
   {
      int linearEqualityConstraintsSize = getLinearEqualityConstraintsSize();
      int maxIterations = 100;
      int iterations = 0;

      // precomputed constants over iterations
      wInverse.reshape(quadraticCostGMatrix.numRows, quadraticCostGMatrix.numCols);
      CommonOps.invert(quadraticCostGMatrix, wInverse);    // can be sped up as w is block-diagonal

      gVector.reshape(quadraticCostFVector.numRows, quadraticCostFVector.numCols);
      gVector.set(quadraticCostFVector);

      wInverseG.reshape(wInverse.numRows, 1);
      CommonOps.mult(wInverse, gVector, wInverseG);

      if (linearEqualityConstraintsSize > 0)
      {
         rMatrix.reshape(linearEqualityConstraintsSize, numberOfVariablesToSolve);
         CommonOps.insert(linearEqualityConstraintA, rMatrix, 0, 0);
         eVector.reshape(linearEqualityConstraintsSize, 1);
         CommonOps.insert(linearEqualityConstraintB, eVector, 0, 0);
      }

      solutionVector.reshape(numberOfVariablesToSolve, 1);

      boolean done = false;

      while (!done)
      {
         int activeInequalityConstraintSize = getActiveSetSize();
         int activeConstraintSize = linearEqualityConstraintsSize + activeInequalityConstraintSize;
         alphaAndGamma.reshape(activeConstraintSize, 1);

         if ((linearEqualityConstraintsSize > 0) || (activeInequalityConstraintSize > 0))    // if there is any constraints
         {
            // rMatrix = [linearEqualityConstraintMatrix; activeLinearInequalityConstraintMatrix];
            rMatrix.reshape(activeConstraintSize, numberOfVariablesToSolve, true);
            if (activeInequalityConstraintSize > 0)
               setPartialMatrixForInequalityConstraints(linearInequalityConstraintA, linearInequalityActiveSet, linearEqualityConstraintsSize, 0, rMatrix);

            // eVector = [linearEqualityConstraintVector; activeLinearInequalityConstraintVector];
            eVector.reshape(activeConstraintSize, 1, true);
            if (activeInequalityConstraintSize > 0)
               setPartialVectorForInequalityConstraints(linearInequalityConstraintB, linearInequalityActiveSet, linearEqualityConstraintsSize, eVector);

            // wInverse * R'
            wInverseRTranspose.reshape(numberOfVariablesToSolve, activeConstraintSize);
            CommonOps.multTransB(wInverse, rMatrix, wInverseRTranspose);

            // LHS
            leftSide.reshape(activeConstraintSize, activeConstraintSize);
            CommonOps.mult(-1, rMatrix, wInverseRTranspose, leftSide);

            // RHS
            rightSide.reshape(activeConstraintSize, 1);
            rightSide.set(eVector);
            CommonOps.multAddTransA(wInverseRTranspose, gVector, rightSide);
            CommonOps.solve(leftSide, rightSide, alphaAndGamma);

            // solve z from alphaAndGamma
            solutionVector.set(wInverseG);
            CommonOps.multAddTransA(-1, rMatrix, alphaAndGamma, solutionVector);
         }

         else
         {
            CommonOps.mult(-1, wInverse, gVector, solutionVector);
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

                  double pz = MatrixTools.multMatrixRowVector(linearInequalityConstraintA, i, solutionVector);
                  if (pz > linearInequalityConstraintB.get(i, 0))
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

      return solutionVector.getData();
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
