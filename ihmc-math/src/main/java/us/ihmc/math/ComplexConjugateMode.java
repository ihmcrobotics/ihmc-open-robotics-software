package us.ihmc.math;

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

      if ((Math.abs(dotProduct.real() - 1.0) > 1e-7) || (Math.abs(dotProduct.imaginary() - 0.0) > 1e-7))
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

      for (int i = 0; i < leftEigenvectorV.length; i++)
      {
         ret[i] = new ComplexNumber(leftEigenvectorV[i]);
      }

      return ret;
   }

   public ComplexNumber[] getRightEigenvectorWCopy()
   {
      ComplexNumber[] ret = new ComplexNumber[rightEigenvectorW.length];

      for (int i = 0; i < rightEigenvectorW.length; i++)
      {
         ret[i] = new ComplexNumber(rightEigenvectorW[i]);
      }

      return ret;
   }

   public double[] getZetaAndOmega()
   {
      return computeZetaAndOmega(eigenvalue);
   }

   public static double[] computeZetaAndOmega(ComplexNumber eigenValue)
   {
      double omega = eigenValue.magnitude();
      double zeta = -eigenValue.real() / omega;

      return new double[] {zeta, omega};
   }

   public String toString()
   {
      return "ComplexConjugateMode: eigenvalue = " + eigenvalue + ", omega = " + getZetaAndOmega()[1] + ", zeta = " + getZetaAndOmega()[0];
   }
}
