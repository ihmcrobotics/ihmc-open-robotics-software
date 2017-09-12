package us.ihmc.robotics.linearDynamicSystems;

import Jama.Matrix;
import us.ihmc.robotics.dataStructures.ComplexNumber;

public class SingleRealMode
{
   private final double eigenvalue;
   private final Matrix leftEigenvectorV;
   private final Matrix rightEigenvectorW;

   public SingleRealMode(ComplexNumber eigenvalue, ComplexNumber[] leftEigenvectorV, ComplexNumber[] rightEigenvectorW)
   {
      @SuppressWarnings("unused")
      double realEigenvalue = verifyRealAndReturnRealPart(eigenvalue);
      double[] realV = verifyRealAndReturnRealPart(leftEigenvectorV);
      double[] realW = verifyRealAndReturnRealPart(rightEigenvectorW);

      verifySameLength(realV, realW);
      verifyDotProductEqualsOne(realV, realW);

      if (Math.abs(eigenvalue.imag()) > 1e-7)
         throw new RuntimeException("Eigenvalue must be real!");

      this.eigenvalue = eigenvalue.real();
      this.leftEigenvectorV = createSingleColumnMatrix(realV);
      this.rightEigenvectorW = createSingleRowMatrix(realW);
   }

   private double[] verifyRealAndReturnRealPart(ComplexNumber[] complexArray)
   {
      int length = complexArray.length;
      double[] ret = new double[length];

      for (int i = 0; i < length; i++)
      {
         ret[i] = verifyRealAndReturnRealPart(complexArray[i]);
      }

      return ret;
   }

   private double verifyRealAndReturnRealPart(ComplexNumber complexNumber)
   {
      if (Math.abs(complexNumber.imag()) > 1e-6)
      {
         throw new RuntimeException("Should only be real for SingleRealMode!. complexNumber = " + complexNumber);
      }

      return complexNumber.real();
   }

   public SingleRealMode(double eigenvalue, double[] leftEigenvectorV, double[] rightEigenvectorW)
   {
      verifySameLength(leftEigenvectorV, rightEigenvectorW);
      verifyDotProductEqualsOne(leftEigenvectorV, rightEigenvectorW);

      this.eigenvalue = eigenvalue;
      this.leftEigenvectorV = createSingleColumnMatrix(leftEigenvectorV);
      this.rightEigenvectorW = createSingleRowMatrix(rightEigenvectorW);
   }



   private void verifySameLength(double[] leftEigenvectorV, double[] rightEigenvectorW)
   {
      if (leftEigenvectorV.length != rightEigenvectorW.length)
      {
         throw new IllegalArgumentException("leftEigenvectorV.length != rightEigenvectorW.length");
      }
   }

   private void verifyDotProductEqualsOne(double[] leftEigenvectorV, double[] rightEigenvectorW)
   {
      double dotProduct = 0.0;
      for (int i = 0; i < leftEigenvectorV.length; i++)
      {
         dotProduct += leftEigenvectorV[i] * rightEigenvectorW[i];
      }

      if (Math.abs(dotProduct - 1.0) > 1e-7)
      {
         throw new IllegalArgumentException("leftEigenvectorV.dot(rightEigenvectorW) must be 1.0!");
      }
   }

   private Matrix createSingleColumnMatrix(double[] values)
   {
      int numRows = values.length;
      Matrix ret = new Matrix(numRows, 1);

      for (int i = 0; i < numRows; i++)
      {
         ret.set(i, 0, values[i]);
      }

      return ret;
   }

   private Matrix createSingleRowMatrix(double[] values)
   {
      int numColumns = values.length;
      Matrix ret = new Matrix(1, numColumns);

      for (int i = 0; i < numColumns; i++)
      {
         ret.set(0, i, values[i]);
      }

      return ret;
   }

   public double getEigenvalue()
   {
      return eigenvalue;
   }
   
   public Matrix getLeftEigenvectorVCopy()
   {      
      int numRows = leftEigenvectorV.getRowDimension();
      Matrix ret = new Matrix(numRows, 1);

      getLeftEigenvectorV(ret);
      
      return ret;
   }
   
   public void getLeftEigenvectorV(Matrix leftEigenvectorToPack)
   {      
      int numRows = leftEigenvectorV.getRowDimension();

      for (int i = 0; i < numRows; i++)
      {
         leftEigenvectorToPack.set(i, 0, leftEigenvectorV.get(i, 0));
      }
   }
   
   public Matrix getRightEigenvectorWCopy()
   {      
      int numColumns = rightEigenvectorW.getColumnDimension();
      Matrix ret = new Matrix(1, numColumns);

      getRightEigenvectorW(ret);
      
      return ret;
   }
   
   public void getRightEigenvectorW(Matrix rightEigenvectorToPack)
   {      
      int numColumns = rightEigenvectorW.getColumnDimension();

      for (int i = 0; i < numColumns; i++)
      {
         rightEigenvectorToPack.set(0, i, rightEigenvectorW.get(0, i));
      }
   }

   public TransferFunctionMatrix constructTransferFunctionMatrix()
   {
      Matrix vwT = leftEigenvectorV.times(rightEigenvectorW);

      double[] denominator = new double[] {1.0, -eigenvalue};

      int numRows = vwT.getRowDimension();
      int numColumns = vwT.getColumnDimension();
      TransferFunction[][] transferFunctions = new TransferFunction[numRows][numColumns];

      for (int i = 0; i < numRows; i++)
      {
         for (int j = 0; j < numColumns; j++)
         {
            double[] numerator = new double[] {vwT.get(i, j)};
            TransferFunction transferFunction = new TransferFunction(numerator, denominator);
            transferFunctions[i][j] = transferFunction;
         }
      }

      return new TransferFunctionMatrix(transferFunctions);
   }
   
   public String toString()
   {
      return "RealMode: eigenvalue = " + eigenvalue;
   }
}
