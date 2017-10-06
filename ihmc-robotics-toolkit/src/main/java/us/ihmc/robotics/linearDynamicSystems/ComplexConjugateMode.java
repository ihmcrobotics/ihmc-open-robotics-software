package us.ihmc.robotics.linearDynamicSystems;

import us.ihmc.robotics.dataStructures.ComplexNumber;
import us.ihmc.robotics.dataStructures.Polynomial;

public class ComplexConjugateMode
{
   private final ComplexNumber eigenvalue;
   private final ComplexNumber[] leftEigenvectorV;
   private final ComplexNumber[] rightEigenvectorW;

   public ComplexConjugateMode(ComplexNumber eigenvalue, ComplexNumber[] leftEigenvectorV, ComplexNumber[] rightEigenvectorW)
   {
      verifySameLength(leftEigenvectorV, rightEigenvectorW);
      verifyDotProductEqualsOne(leftEigenvectorV, rightEigenvectorW);

      this.eigenvalue = eigenvalue;
      this.leftEigenvectorV = leftEigenvectorV;
      this.rightEigenvectorW = rightEigenvectorW;
   }

   private void verifySameLength(ComplexNumber[] leftEigenvectorV, ComplexNumber[] rightEigenvectorW)
   {
      if (leftEigenvectorV.length != rightEigenvectorW.length)
      {
         throw new IllegalArgumentException("leftEigenvectorV.length != rightEigenvectorW.length");
      }
   }

   private void verifyDotProductEqualsOne(ComplexNumber[] leftEigenvectorV, ComplexNumber[] rightEigenvectorW)
   {
      ComplexNumber dotProduct = new ComplexNumber(0.0, 0.0);

      for (int i = 0; i < leftEigenvectorV.length; i++)
      {
         dotProduct = dotProduct.plus(leftEigenvectorV[i].times(rightEigenvectorW[i]));
      }

      if ((Math.abs(dotProduct.real() - 1.0) > 1e-7) || (Math.abs(dotProduct.imag() - 0.0) > 1e-7))
      {
         throw new IllegalArgumentException("leftEigenvectorV.dot(rightEigenvectorW) must be 1.0!");
      }
   }

   public ComplexNumber getEigenvalue()
   {
      return eigenvalue;
   }
   
   public ComplexNumber[] getLeftEigenvectorVCopy()
   {
      ComplexNumber[] ret = new ComplexNumber[leftEigenvectorV.length];
      
      for (int i=0; i<leftEigenvectorV.length; i++)
      {
         ret[i] = new ComplexNumber(leftEigenvectorV[i]);
      }
      
      return ret;
   }
   
   public double[] getZetaAndOmega()
   {
      double omega = eigenvalue.magnitude();
      double zeta = -eigenvalue.real()/omega;
      
      return new double[]{zeta, omega};
   }

   public TransferFunctionMatrix constructTransferFunctionMatrix()
   {
      int order = leftEigenvectorV.length;
      TransferFunction[][] transferFunctions = new TransferFunction[order][order];

      Polynomial denominatorPolynomial = new Polynomial(new double[] {1.0, -2.0 * eigenvalue.real(), eigenvalue.magnitudeSquared()});

      for (int i = 0; i < order; i++)
      {
         for (int j = 0; j < order; j++)
         {
            ComplexNumber Rij = leftEigenvectorV[i].times(rightEigenvectorW[j]);

            double[] numeratorCoefficients = new double[] {2.0 * Rij.real(), -2.0 * eigenvalue.real() * Rij.real() - 2.0 * eigenvalue.imag() * Rij.imag()};
            Polynomial numeratorPolynomial = new Polynomial(numeratorCoefficients);

            transferFunctions[i][j] = new TransferFunction(numeratorPolynomial, denominatorPolynomial);
         }
      }

      return new TransferFunctionMatrix(transferFunctions);
   }

   public String toString()
   {
      return "ComplexConjugateMode: eigenvalue = " + eigenvalue + ", omega = " + getZetaAndOmega()[1] + ", zeta = " + getZetaAndOmega()[0];
   }

}
