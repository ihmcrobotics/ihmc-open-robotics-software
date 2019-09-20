package us.ihmc.robotics.linearDynamicSystems;

import Jama.Matrix;

public class TransferFunctionToStateSpaceConverter
{

   public static LinearDynamicSystem convertTransferFunctionToStateSpaceObservableCanonicalForm(TransferFunction transferFunction)
   {
      double[] numeratorCoefficients = transferFunction.getNumeratorCoefficients();
      double[] denominatorCoefficients = transferFunction.getDenominatorCoefficients();

      int systemOrder = denominatorCoefficients.length - 1;
      numeratorCoefficients = padZerosOnFront(numeratorCoefficients, denominatorCoefficients.length);

      double a0 = denominatorCoefficients[0];
      for (int i = 0; i < systemOrder; i++)
      {
         denominatorCoefficients[i] = denominatorCoefficients[i] / a0;
      }

      double[][] elementsA = new double[systemOrder][systemOrder];

      for (int i = 0; i < systemOrder; i++)
      {
         for (int j = 0; j < systemOrder; j++)
         {
            if (j == i + 1)
               elementsA[i][j] = 1.0;
            else if (j == 0)
               elementsA[i][j] = -denominatorCoefficients[i + 1];
            else
               elementsA[i][j] = 0.0;
         }
      }

      double[][] elementsB = new double[systemOrder][1];
      double b0 = numeratorCoefficients[0];

      for (int i = 0; i < systemOrder; i++)
      {
         elementsB[i][0] = numeratorCoefficients[i + 1] - denominatorCoefficients[i + 1] * b0;
      }

      double[][] elementsC = new double[1][systemOrder];
      for (int i = 0; i < systemOrder; i++)
      {
         if (i == 0)
            elementsC[0][i] = 1.0;
         else
            elementsC[0][i] = 0.0;
      }
      double[][] elementsD = new double[1][1];
      elementsD[0][0] = b0;

      Matrix matrixA = new Matrix(elementsA);
      Matrix matrixB = new Matrix(elementsB);
      Matrix matrixC = new Matrix(elementsC);
      Matrix matrixD = new Matrix(elementsD);

      LinearDynamicSystem linearDynamicSystem = new LinearDynamicSystem(matrixA, matrixB, matrixC, matrixD);

      return linearDynamicSystem;
   }

   private static double[] padZerosOnFront(double[] coefficients, int newLength)
   {
      double[] newCoefficients = new double[newLength];
      int numberOfPaddedZeros = newCoefficients.length - coefficients.length;

      for (int i = 0; i < newLength; i++)
      {
         if (i < numberOfPaddedZeros)
         {
            newCoefficients[i] = 0.0;
         }
         else
         {
            newCoefficients[i] = coefficients[i - numberOfPaddedZeros];
         }
      }

      return newCoefficients;
   }
}
