package us.ihmc.math;

import java.util.List;

/**
 * Garbage free implementation of the FFT algorithm
 */
public class FastFourierTransform
{
   private static final double logTwo = Math.log(2.0);

   private boolean isTransformUpToDate = false;
   private boolean isInverseTransformUpToDate = false;

   private final int maxNumberOfCoefficients;
   private final int logMaxNumberOfCoefficients;
   private int numberOfCoefficients;
   private final List<ComplexNumber> rootsOfUnity;
   private final ComplexNumber[] coefficients;
   private final ComplexNumber[] transformedCoeffs;
   private final ComplexNumber[] inverseTransformedCoeffs;

   public FastFourierTransform(int maxNumberOfCoefficients)
   {
      this.logMaxNumberOfCoefficients = (int) Math.ceil(Math.log(maxNumberOfCoefficients) / logTwo);
      this.maxNumberOfCoefficients = (int) Math.pow(2, logMaxNumberOfCoefficients);
      this.rootsOfUnity = RootsOfUnity.getRootsOfUnity(this.maxNumberOfCoefficients);
      this.coefficients = new ComplexNumber[this.maxNumberOfCoefficients];
      this.transformedCoeffs = new ComplexNumber[this.maxNumberOfCoefficients];
      this.inverseTransformedCoeffs = new ComplexNumber[this.maxNumberOfCoefficients];
      for (int i = 0; i < this.maxNumberOfCoefficients; i++)
      {
         coefficients[i] = new ComplexNumber();
         transformedCoeffs[i] = new ComplexNumber();
         inverseTransformedCoeffs[i] = new ComplexNumber();
      }
   }

   /**
    * Sets the array of coefficients of which we want to compute the transform.
    * @param coefficients coefficient values.
    */
   public void setCoefficients(ComplexNumber[] coefficients)
   {
      isTransformUpToDate = false;
      isInverseTransformUpToDate = false;

      numberOfCoefficients = coefficients.length;
      if (numberOfCoefficients > maxNumberOfCoefficients)
         throw new RuntimeException("Insufficient number of coefficients for FFT transform, max: " + maxNumberOfCoefficients + ", provided: "
               + numberOfCoefficients);
      int index = 0;
      for (; index < numberOfCoefficients; index++)
      {
         this.coefficients[index].set(coefficients[index]);
      }
      for (; index < maxNumberOfCoefficients; index++)
      {
         this.coefficients[index].setToPurelyReal(0.0);
      }
   }

   /**
    * Sets the array of coefficients of which we want to compute the transform. This sets them to real only values.
    * @param coefficients real only coefficient values.
    */
   public void setCoefficients(double[] coefficients)
   {
      isTransformUpToDate = false;
      isInverseTransformUpToDate = false;

      numberOfCoefficients = coefficients.length;
      if (numberOfCoefficients > maxNumberOfCoefficients)
         throw new RuntimeException("Insufficient number of coefficients for FFT transform, max: " + maxNumberOfCoefficients + ", provided: "
               + numberOfCoefficients);
      int index = 0;
      for (; index < numberOfCoefficients; index++)
      {
         this.coefficients[index].setToPurelyReal(coefficients[index]);
      }
      for (; index < maxNumberOfCoefficients; index++)
      {
         this.coefficients[index].setToPurelyReal(0.0);
      }
   }

   private final ComplexNumber tempComplex1 = new ComplexNumber(), tempComplex2 = new ComplexNumber(), tempComplex = new ComplexNumber();

   public ComplexNumber[] getForwardTransform()
   {
      if (!isTransformUpToDate)
         computeFastFourierTransform();
      return transformedCoeffs;
   }

   public ComplexNumber[] getInverseTransform()
   {
      if (!isInverseTransformUpToDate)
         computeInverseFastFourierTransform();
      return inverseTransformedCoeffs;
   }

   private void computeFastFourierTransform()
   {
      bitReverseCopy(transformedCoeffs, coefficients);

      int m = 1;

      for (int i = 1; i <= logMaxNumberOfCoefficients; i++)
      {
         int half_m = m;
         m = m << 1; // Computing m = 2^i

         tempComplex.setToPurelyReal(1.0);
         for (int j = 0; j < half_m; j++)
         {
            for (int k = j; k < maxNumberOfCoefficients - 1; k += m)
            {
               tempComplex1.times(tempComplex, transformedCoeffs[k + half_m]);
               tempComplex2.set(transformedCoeffs[k]);
               transformedCoeffs[k].add(tempComplex1, tempComplex2);
               transformedCoeffs[k + half_m].minus(tempComplex2, tempComplex1);
            }
            tempComplex.timesEquals(rootsOfUnity.get(maxNumberOfCoefficients - maxNumberOfCoefficients / m));
         }
      }
      isTransformUpToDate = true;
   }

   private void computeInverseFastFourierTransform()
   {
      bitReverseCopy(inverseTransformedCoeffs, coefficients);

      int m = 1;

      for (int i = 1; i <= logMaxNumberOfCoefficients; i++)
      {
         int half_m = m;
         m = m << 1; // Computing m = 2^i

         tempComplex.setToPurelyReal(1.0);
         for (int j = 0; j < half_m; j++)
         {
            for (int k = j; k < maxNumberOfCoefficients - 1; k += m)
            {
               tempComplex1.times(tempComplex, inverseTransformedCoeffs[k + half_m]);
               tempComplex2.set(inverseTransformedCoeffs[k]);
               inverseTransformedCoeffs[k].add(tempComplex1, tempComplex2);
               inverseTransformedCoeffs[k + half_m].minus(tempComplex2, tempComplex1);
            }
            tempComplex.timesEquals(rootsOfUnity.get(maxNumberOfCoefficients / m));
         }
      }

      for (int i = 0; i < inverseTransformedCoeffs.length; i++)
         inverseTransformedCoeffs[i].scale(1.0 / maxNumberOfCoefficients);

      isInverseTransformUpToDate = true;
   }

   private static void bitReverseCopy(ComplexNumber[] arrayToPack, ComplexNumber[] arrayToCopy)
   {
      int temp = (int) (Math.log(arrayToCopy.length) / logTwo);
      for (int i = 0; i < arrayToCopy.length; i++)
      {
         arrayToPack[i].set(arrayToCopy[bitReverse(i, temp)]);
      }
   }

   static int bitReverse(int a, int numberOfBits)
   {
      int temp = 0;
      for (; numberOfBits > 0; numberOfBits--, temp += a % 2, a /= 2)
         temp <<= 1;
      return temp;
   }
}
