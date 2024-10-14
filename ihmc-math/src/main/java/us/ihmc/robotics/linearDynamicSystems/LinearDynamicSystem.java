package us.ihmc.robotics.linearDynamicSystems;

import Jama.Matrix;
import org.ejml.simple.SimpleMatrix;
import us.ihmc.robotics.math.trajectories.core.Polynomial;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialReadOnly;

import java.util.ArrayList;

public class LinearDynamicSystem
{
   private final SimpleMatrix matrixA;
   private SimpleMatrix matrixB, matrixC, matrixD;
   private final TransferFunctionMatrix sIMinusAInverse;

   public LinearDynamicSystem(SimpleMatrix matrixA, SimpleMatrix matrixB, SimpleMatrix matrixC, SimpleMatrix matrixD)
   {
      if (matrixA == null)
      {
         throw new RuntimeException("matrixA must be defined. B,C,D can be null");
      }

      int order = matrixA.numRows();

      if (matrixA.numCols() != order)
      {
         throw new RuntimeException("matrixA must be square!");
      }

      this.matrixA = new SimpleMatrix(matrixA);
      setMatrixB(matrixB);
      setMatrixC(matrixC);
      setMatrixD(matrixD);
      sIMinusAInverse = computeSIMinusAInverse();
   }

   public SimpleMatrix getMatrixA()
   {
      return new SimpleMatrix(matrixA);
   }

   public SimpleMatrix getMatrixB()
   {
      return new SimpleMatrix(matrixB);
   }

   public SimpleMatrix getMatrixC()
   {
      return new SimpleMatrix(matrixC);
   }

   public SimpleMatrix getMatrixD()
   {
      return new SimpleMatrix(matrixD);
   }

   public LinearDynamicSystem addFullStateFeedback(SimpleMatrix matrixK)
   {
      if (matrixB == null)
      {
         throw new RuntimeException("Matrix B must not be null for addFullStateFeedback!");
      }

      SimpleMatrix newMatrixA = matrixA.plus(matrixB.mult(matrixK.scale(-1.0)));
      SimpleMatrix newMatrixB = matrixB.copy();
      SimpleMatrix newMatrixC = null, newMatrixD = null;

      if (matrixC != null)
      {
         newMatrixC = matrixC;

         if (matrixD != null)
         {
            newMatrixC = matrixC.plus(matrixD.mult(matrixK.scale(-1.0)));
            newMatrixD = matrixD.copy();
         }
      }

      return new LinearDynamicSystem(newMatrixA, newMatrixB, newMatrixC, newMatrixD);
   }

   public LinearDynamicSystem addOutputStateFeedback(SimpleMatrix matrixK)
   {
      return addOutputStateFeedback(matrixK, null);
   }

   public LinearDynamicSystem addOutputStateFeedback(SimpleMatrix matrixK, SimpleMatrix matrixFF)
   {
      if (matrixB == null)
      {
         throw new RuntimeException("Matrix B must not be null for addOutputStateFeedback!");
      }

      if (matrixD != null)
      {
         throw new RuntimeException("Matrix D must be null for addOutputStateFeedback!");
      }

      if (matrixC == null)
      {
         throw new RuntimeException("Matrix C must not be null for addOutputStateFeedback!");
      }

      SimpleMatrix newMatrixA = matrixA.plus(matrixB.mult(matrixK.scale(-1.0)).mult(matrixC));
      SimpleMatrix newMatrixB = matrixB.copy();
      SimpleMatrix newMatrixC = matrixC.copy();

      if (matrixFF != null)
      {
         newMatrixB = newMatrixB.mult(matrixFF);
      }

      return new LinearDynamicSystem(newMatrixA, newMatrixB, newMatrixC, null);
   }

   public TransferFunctionMatrix getTransferFunctionMatrix()
   {

      // Returns C(sI-A)^(-1)B + D.
      // If any of C, B, or D are null, then just skip the multiplication.
      TransferFunctionMatrix ret = sIMinusAInverse;

      if (matrixC != null)
      {
         ret = ret.preMultiply(matrixC);
      }

      if (matrixB != null)
      {
         ret = ret.times(matrixB);
      }

      if (matrixD != null)
      {
         ret = ret.plus(matrixD);
      }

      return ret;
   }

   public int getOrder()
   {
      return matrixA.numRows();
   }

   public int getInputSize()
   {
      return matrixB.numCols();
   }

   public int getOutputSize()
   {
      return matrixC.numRows();
   }

   public void setMatrixB(SimpleMatrix matrixB)
   {
      if (matrixB != null)
      {
         this.matrixB = new SimpleMatrix(matrixB);
      }
   }

   public void setMatrixC(SimpleMatrix matrixC)
   {
      if (matrixC != null)
      {
         this.matrixC = new SimpleMatrix(matrixC);
      }
   }

   public void setMatrixD(SimpleMatrix matrixD)
   {
      if (matrixD != null)
      {
         this.matrixD = new SimpleMatrix(matrixD);
      }
   }

   private TransferFunctionMatrix computeSIMinusAInverse()
   {
      return computeSIMinusAInverseUsingCofactors();
//
//       return computeSIMinusAInverseUsingEigenvalueDecomposition();
   }

   private TransferFunctionMatrix computeSIMinusAInverseUsingCofactors()
   {
      int order = matrixA.numCols();
      PolynomialMatrix sIMinusA = PolynomialMatrix.constructSIMinusA(matrixA);
      PolynomialReadOnly characteristicEquation = sIMinusA.computeDeterminant();
      PolynomialReadOnly[][] numerators = new Polynomial[order][order];

      for (int i = 0; i < order; i++)
      {
         for (int j = 0; j < order; j++)
         {
            numerators[j][i] = sIMinusA.computeCofactor(i, j);
         }
      }

      return new TransferFunctionMatrix(numerators, characteristicEquation);
   }

   private static Matrix toJamaMatrix(SimpleMatrix matrix)
   {
      Matrix ret = new Matrix(matrix.numRows(), matrix.numCols());
      for (int row = 0; row < matrix.numRows(); row++)
      {
         for (int col = 0; col < matrix.numCols(); col++)
            ret.set(row, col, matrix.get(row, col));
      }

      return ret;
   }

   @SuppressWarnings("unused")
   private TransferFunctionMatrix computeSIMinusAInverseUsingEigenvalueDecomposition()
   {
      EigenvalueDecomposer eigenvalueDecomposer = new EigenvalueDecomposer(toJamaMatrix(matrixA));
      ArrayList<SingleRealMode> realModes = eigenvalueDecomposer.getRealModes();
      ArrayList<ComplexConjugateMode> complexConjugateModes = eigenvalueDecomposer.getComplexConjugateModes();
      TransferFunctionMatrix ret = null;

      for (SingleRealMode realMode : realModes)
      {
         TransferFunctionMatrix realModeMatrix = realMode.constructTransferFunctionMatrix();

         if (ret == null)
         {
            ret = realModeMatrix;
         }
         else
         {
            ret = ret.plus(realModeMatrix);
         }
      }

      for (ComplexConjugateMode complexConjugateMode : complexConjugateModes)
      {
         TransferFunctionMatrix complexConjugateModeMatrix = complexConjugateMode.constructTransferFunctionMatrix();

         if (ret == null)
         {
            ret = complexConjugateModeMatrix;
         }
         else
         {
            ret = ret.plus(complexConjugateModeMatrix);
         }
      }

      return ret;
   }

   public double[] eulerIntegrateOneStep(double[] currentState, double[] input, double stepSize)
   {
      SimpleMatrix currentStateMatrix = new SimpleMatrix(currentState.length, 1, true, currentState);
      SimpleMatrix inputMatrix = new SimpleMatrix(input.length, 1, true, input);

      SimpleMatrix nextStateMatrix = eulerIntegrateOneStep(currentStateMatrix, inputMatrix, stepSize);

      double[] nextState = new double[currentState.length];
      copyArray(nextStateMatrix, nextState);

      return nextState;
   }

   public SimpleMatrix eulerIntegrateOneStep(SimpleMatrix currentState, SimpleMatrix input, double stepSize)
   {
      SimpleMatrix nextState = currentState.copy();

      SimpleMatrix aTimesX = matrixA.mult(currentState);

      SimpleMatrix bTimesU = matrixB.mult(input);
      SimpleMatrix aTimesXPlusBTimesU = aTimesX.plus(bTimesU);

      SimpleMatrix aTimesXPlusBTimesUTimesStepSize = aTimesXPlusBTimesU.scale(stepSize);

      nextState = nextState.plus(aTimesXPlusBTimesUTimesStepSize);

      return nextState;
   }

   public SimpleMatrix getOutputFromState(SimpleMatrix state, SimpleMatrix input)
   {
      SimpleMatrix cTimesX = matrixC.mult(state);
      SimpleMatrix dTimesU = matrixD.mult(input);

      return cTimesX.plus(dTimesU);
   }

   public double[] getOutputFromState(double[] state, double[] input)
   {
      SimpleMatrix stateMatrix = new SimpleMatrix(state.length, 1, true, state);
      SimpleMatrix inputMatrix = new SimpleMatrix(input.length, 1, true, input);

      SimpleMatrix outputMatrix = getOutputFromState(stateMatrix, inputMatrix);

      double[] output = new double[outputMatrix.numRows()];
      copyArray(outputMatrix, output);

      return output;
   }

   public double[][] simulateInitialConditions(double[] initialConditions, double stepSize, int numTicks)
   {
      int order = matrixA.numRows();

      if (initialConditions.length != order)
      {
         throw new RuntimeException("initialConditions.length != order");
      }

      // Just use Euler integrations for now:
      double[][] ret = new double[numTicks][order];
      SimpleMatrix state = new SimpleMatrix(order, 1);
      copyArray(initialConditions, state);

      for (int i = 0; i < numTicks; i++)
      {
         copyArray(state, ret[i]);

         SimpleMatrix aTimesX = matrixA.mult(state);
         SimpleMatrix aTimesXTimesStepSize = aTimesX.scale(stepSize);

         state = state.plus(aTimesXTimesStepSize);
      }

      return ret;
   }

   @SuppressWarnings("unused")
   private void copyArray(double[] in, double[] out)
   {
      for (int i = 0; i < in.length; i++)
      {
         out[i] = in[i];
      }
   }

   private void copyArray(SimpleMatrix vectorIn, double[] out)
   {
      for (int i = 0; i < vectorIn.numRows(); i++)
      {
         out[i] = vectorIn.get(i, 0);
      }
   }

   private void copyArray(double[] in, SimpleMatrix vectorOut)
   {
      for (int i = 0; i < vectorOut.numRows(); i++)
      {
         vectorOut.set(i, 0, in[i]);
      }
   }

   @Override
   public String toString()
   {
      return "A = " + matrixA.toString() + "\nB = " + matrixB.toString() + "\nC = " + matrixC.toString() + "\nD = " + matrixD.toString();
   }
}
