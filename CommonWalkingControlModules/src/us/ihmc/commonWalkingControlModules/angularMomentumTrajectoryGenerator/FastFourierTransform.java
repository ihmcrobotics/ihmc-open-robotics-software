package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import org.junit.Test;

import us.ihmc.commons.PrintTools;

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
   
   public FastFourierTransform()
   {
      this(2048);
   }
   
   public FastFourierTransform(int maxNumberOfCoefficients)
   {
      
   }
}
