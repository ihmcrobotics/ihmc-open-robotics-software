package us.ihmc.robotics.math;

import us.ihmc.robotics.dataStructures.ComplexNumber;

import java.util.List;

/**
 * Garbage free implementation of the FFT algorithm
 */
public class FastFourierTransform
{
   /**
    * standard FFT object available for all to use
    */
   public static final FastFourierTransform fft = new FastFourierTransform(8);

   public static FastFourierTransform getFourierTransformer()
   {
      return fft;
   }

   private int maxNumberOfCoefficients;
   private int logMaxNumberOfCoefficients;
   private int numberOfCoefficients;
   private List<ComplexNumber> rootsOfUnity;
   private ComplexNumber[] coefficients;
   private ComplexNumber[] transformedCoeffs;

   public FastFourierTransform(int maxNumberOfCoefficients)
   {
      this.logMaxNumberOfCoefficients = (int) Math.ceil(Math.log(maxNumberOfCoefficients) / Math.log(2.0));
      this.maxNumberOfCoefficients = (int) Math.pow(2, logMaxNumberOfCoefficients);
      this.rootsOfUnity = RootsOfUnity.getRootsOfUnity(this.maxNumberOfCoefficients);
      this.coefficients = new ComplexNumber[this.maxNumberOfCoefficients];
      this.transformedCoeffs = new ComplexNumber[this.maxNumberOfCoefficients];
      for (int i = 0; i < this.maxNumberOfCoefficients; i++)
      {
         coefficients[i] = new ComplexNumber();
         transformedCoeffs[i] = new ComplexNumber();
      }
   }

   public void setCoefficients(ComplexNumber[] coefficients)
   {
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

   public void setCoefficients(double[] coefficients)
   {
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

   public void setCoefficients(double[] coefficients, int numberOfCoefficientsToUse)
   {
      numberOfCoefficients = numberOfCoefficientsToUse;
      if (numberOfCoefficientsToUse > maxNumberOfCoefficients)
         throw new RuntimeException("Insufficient number of coefficients for FFT transform, max: " + maxNumberOfCoefficients + ", provided: "
               + numberOfCoefficientsToUse);
      int index = 0;
      for (; index < numberOfCoefficientsToUse; index++)
      {
         this.coefficients[index].setToPurelyReal(coefficients[index]);
      }
      for (; index < maxNumberOfCoefficients; index++)
      {
         this.coefficients[index].setToPurelyReal(0.0);
      }
   }
   
   private ComplexNumber tempComplex1 = new ComplexNumber(), tempComplex2 = new ComplexNumber(), tempComplex = new ComplexNumber();

   public ComplexNumber[] getForwardTransform()
   {
      transform(false);
      return transformedCoeffs;
   }

   public ComplexNumber[] getInverseTransform()
   {
      transform(true);
      return transformedCoeffs;
   }

   private void transform(boolean inverse)
   {
      bitReverseCopy(transformedCoeffs, coefficients);
      for (int i = 1; i <= logMaxNumberOfCoefficients; i++)
      {
         int m = (int) Math.pow(2.0, i);
         tempComplex.setToPurelyReal(1.0);
         for (int j = 0; j < m / 2; j++)
         {
            for (int k = j; k < maxNumberOfCoefficients - 1; k += m)
            {
               tempComplex1.timesAndStore(tempComplex, transformedCoeffs[k + m / 2]);
               tempComplex2.set(transformedCoeffs[k]);
               transformedCoeffs[k].plusAndStore(tempComplex1, tempComplex2);
               transformedCoeffs[k + m / 2].minusAndStore(tempComplex2, tempComplex1);
            }
            if (!inverse)
               tempComplex.timesAndStore(rootsOfUnity.get(maxNumberOfCoefficients - maxNumberOfCoefficients / m));
            else
               tempComplex.timesAndStore(rootsOfUnity.get(maxNumberOfCoefficients / m));
         }
      }
      if (inverse)
      {
         for (int i = 0; i < transformedCoeffs.length; i++)
            transformedCoeffs[i].scale(1.0 / maxNumberOfCoefficients);
      }
   }

   private void bitReverseCopy(ComplexNumber[] arrayToPack, ComplexNumber[] arrayToCopy)
   {
      int temp = (int) (Math.log(arrayToCopy.length) / Math.log(2.0));
      for (int i = 0; i < arrayToCopy.length; i++)
      {
         arrayToPack[i].set(arrayToCopy[bitReverse(i, temp)]);
      }
   }

   public int bitReverse(int a, int numberOfBits)
   {
      int temp = 0;
      for (; numberOfBits > 0; numberOfBits--, temp += a % 2, a /= 2)
         temp <<= 1;
      return temp;
   }

   private void clearTransformed()
   {
      for (int i = 0; i < maxNumberOfCoefficients; i++)
         transformedCoeffs[i].set(0.0, 0.0);
   }

   private void clearInput()
   {
      for (int i = 0; i < numberOfCoefficients; i++)
         coefficients[i].set(0.0, 0.0);
   }

   public void clear()
   {
      clearTransformed();
      clearInput();
   }

   public int getMaxNumberOfCoefficients()
   {
      return maxNumberOfCoefficients;
   }
   
   public List<ComplexNumber> getRootsOfUnity()
   {
      return rootsOfUnity;
   }
}
