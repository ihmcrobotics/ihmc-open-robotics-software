package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

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
   private List<ComplexNumber> rootsOfUnity;
   private ComplexNumber[] coefficients;
   private ComplexNumber[] transformedCoeffs;
   private ComplexNumber[][] tempValue;
   public FastFourierTransform()
   {
      this(2048);
   }

   public FastFourierTransform(int maxNumberOfCoefficients)
   {
      this.logMaxNumberOfCoefficients = (int) Math.ceil(Math.log(maxNumberOfCoefficients) / Math.log(2.0));
      this.maxNumberOfCoefficients = (int) Math.pow(2, logMaxNumberOfCoefficients);
      this.rootsOfUnity = RootsOfUnity.getRootsOfUnity(this.maxNumberOfCoefficients);
      this.coefficients = new ComplexNumber[this.maxNumberOfCoefficients];
      this.transformedCoeffs = new ComplexNumber[this.maxNumberOfCoefficients];
      this.tempValue = new ComplexNumber[this.logMaxNumberOfCoefficients][this.maxNumberOfCoefficients/2];
      for (int i = 0; i < this.maxNumberOfCoefficients; i++)
      {
         coefficients[i] = new ComplexNumber();
         transformedCoeffs[i] = new ComplexNumber();
      }
      for (int i = 0; i < this.maxNumberOfCoefficients; i++)
         for(int j = 0; j < this.maxNumberOfCoefficients/2; j++)
            tempValue[i][j] = new ComplexNumber();
   }

   public void setCoefficients(double[] coefficients)
   {
      if (coefficients.length > maxNumberOfCoefficients)
         throw new RuntimeException("Insufficient number of coefficients for FFT transform, max: " + maxNumberOfCoefficients + ", provided: "
               + coefficients.length);

      int index = 0;
      for (; index < coefficients.length; index++)
         this.coefficients[index].setToPurelyReal(coefficients[index]);
      for (; index < maxNumberOfCoefficients; index++)
         this.coefficients[index].setToPurelyReal(0.0);
   }

   public ComplexNumber getTransform(ComplexNumber[] coeffs, int N, int k, int Ntotal)
   {
      if(N == 1)
         return coeffs[0];
      else 
      {
         ComplexNumber[] set1 = new ComplexNumber[N/2];
         ComplexNumber[] set2 = new ComplexNumber[N/2]; 
         for(int i = 0 ; i < N; i++)
         {
            if(i%2 == 0)
               set1[i/2] = coeffs[i];
            else
               set2[(i-1)/2] = coeffs[i];
         }
         ComplexNumber a = new ComplexNumber();
         a.multiply(rootsOfUnity.get(k * Ntotal/N), getTransform(coeffs, N/2, k, Ntotal));
         a.add(getTransform(coeffs, N/2, k, Ntotal));
         return a;
      }
   }
   
   public ComplexNumber[] getTransform()
   {
      clearTransformed();
      // Setup the lowest order 
      for(int i = 0; i < this.maxNumberOfCoefficients; i++)
      {
         for(int j=0; j < this.maxNumberOfCoefficients/2; j++)
         {
            tempValue[i][j].multiply(rootsOfUnity.get(index));
         }
      }
      
      for(int i = maxNumberOfCoefficients/2; i!=1; i>>=1)
      {
         for(int j = 0; j < maxNumberOfCoefficients - i; i++)
         {
            
         }
      }
      return transformedCoeffs;
   }

   private void clearTransformed()
   {
      for (int i = 0; i < transformedCoeffs.length; i++)
         transformedCoeffs[i].set(0.0, 0.0);
   }

   private void clearInput()
   {
      for (int i = 0; i < transformedCoeffs.length; i++)
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
}
