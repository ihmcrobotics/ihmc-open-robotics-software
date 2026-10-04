package us.ihmc.robotics.math.filters;

import java.util.Arrays;

/**
 * This class holds the representation of a continuous transfer function of the form:
 * <pre>
 * G(s) = k * (b_0*s^m + b_1*s^{m-1} + ... + b_{m-1}*s + b_m)/(a_0*s^n + a_1*s^{n-1} + ... + a_{n-1}*s + a_n)
 * </pre>
 * where numerator[] = [b_0, b_1, ..., b_{m-1}, b_m] and denominator[] = [a_0, a_1, ..., a_{n-1}, a_n]. This class can
 * be passed into the TransferFunctionDiscretizer to create a discrete representation of that transfer function
 * in the YoFilteredDouble class.
 * In addition, two ContinuousTransferFunctions can be multiplied together to produce a new transfer function of the form
 * <pre>
 *                            N1(s)      N2(s)        N3(s)
 * H3(s) = H1(s)*H2(s) = k1 * ----- k2 * ----- = k3 * -----
 *                            D1(s)      D2(s)        D3(s)
 * </pre>
 *
 * @author Connor Herron
 */
public class ContinuousTransferFunction
{
   private String name;
   private double k;
   private double[] numerator;
   private double[] denominator;

   public ContinuousTransferFunction()
   {
   }

   /**
    * @param k           - gain
    * @param numerator   - numerator coefficients in power order (see above).
    * @param denominator - denominator coefficients in power order (see above).
    */
   public ContinuousTransferFunction(double k, double[] numerator, double[] denominator)
   {
      this("", k, numerator, denominator);
   }

   public ContinuousTransferFunction(String name, double k, double[] numerator, double[] denominator)
   {
      this.name = name;
      this.k = k;
      this.numerator = Arrays.copyOf(numerator, numerator.length);
      this.denominator = Arrays.copyOf(denominator, denominator.length);
   }

   public ContinuousTransferFunction(ContinuousTransferFunction[] severalContinuousTransferFunctions)
   {
      this("", severalContinuousTransferFunctions);
   }

   public ContinuousTransferFunction(String name, ContinuousTransferFunction[] severalContinuousTransferFunctions)
   {
      this.name = name;

      ContinuousTransferFunction resultingTransferFunction = severalContinuousTransferFunctions[0];

      for (int i = 1; i < severalContinuousTransferFunctions.length; i++)
      {
         resultingTransferFunction = multiply(resultingTransferFunction, severalContinuousTransferFunctions[i]);
      }

      this.k = resultingTransferFunction.getGain();
      this.numerator = Arrays.copyOf(resultingTransferFunction.getNumeratorUnsafe(), resultingTransferFunction.getNumeratorUnsafe().length);
      this.denominator = Arrays.copyOf(resultingTransferFunction.getDenominatorUnsafe(), resultingTransferFunction.getDenominatorUnsafe().length);
   }

   /**
    * Creates a new transfer function H3(s) = H1(s) * H2(s).
    * <pre>
    *                            N1(s)      N2(s)        N3(s)
    * H3(s) = H1(s)*H2(s) = k1 * ----- k2 * ----- = k3 * -----
    *                            D1(s)      D2(s)        D3(s)
    * </pre>
    * Allocates a new {@link ContinuousTransferFunction} and new numerator/denominator arrays; not garbage free.
    *
    * @param H1 - Transfer Function 1
    * @param H2 - Transfer Function 2
    * @return H3 - Combined Transfer Function of 1 and 2 (H3(s) = H1(s)*H2(s))
    */
   public static ContinuousTransferFunction multiply(ContinuousTransferFunction H1, ContinuousTransferFunction H2)
   {
      ContinuousTransferFunction result = new ContinuousTransferFunction();
      multiply(result, H1, H2);
      return result;
   }

   /**
    * Packs {@code resultToPack} with H1(s) * H2(s). This resizes {@code resultToPack}'s numerator/denominator
    * arrays to match the product's polynomial order, so it still allocates; it is not garbage free.
    */
   public static void multiply(ContinuousTransferFunction resultToPack, ContinuousTransferFunction H1, ContinuousTransferFunction H2)
   {
      resultToPack.setGain(H1.getGain() * H2.getGain());
      resultToPack.setNumerator(multiplyPolynomials(H1.getNumeratorUnsafe(), H2.getNumeratorUnsafe()));
      resultToPack.setDenominator(multiplyPolynomials(H1.getDenominatorUnsafe(), H2.getDenominatorUnsafe()));
   }

   /**
    * Multiplies two polynomials, poly1(x) and poly2(x), into a newly allocated poly3(x). Not garbage free.
    *
    * @param poly1
    * @param poly2
    * @return poly3 = poly1*poly2
    */
   private static double[] multiplyPolynomials(double[] poly1, double[] poly2)
   {
      double[] poly3 = new double[poly1.length + poly2.length - 1];
      multiplyPolynomials(poly3, poly1, poly2);
      return poly3;
   }

   /**
    * Packs {@code resultToPack} with the coefficients of poly1(x) * poly2(x). {@code resultToPack} must already be
    * sized to {@code poly1.length + poly2.length - 1}. Garbage free.
    */
   private static void multiplyPolynomials(double[] resultToPack, double[] poly1, double[] poly2)
   {
      Arrays.fill(resultToPack, 0.0);

      for (int i = 0; i < poly1.length; i++)
      {
         for (int j = 0; j < poly2.length; j++)
         {
            resultToPack[i + j] += poly1[i] * poly2[j];
         }
      }
   }

   /**
    * Creates a first-order low-pass filter transfer function: G(s) = breakFrequency / (s + breakFrequency).
    */
   public static ContinuousTransferFunction createLowPassFilterTransferFunction(double breakFrequency)
   {
      return new ContinuousTransferFunction("Low-Pass Filter", 1.0, new double[] {breakFrequency}, new double[] {1.0, breakFrequency});
   }

   /**
    * Creates a first-order high-pass filter transfer function: G(s) = s / (s + breakFrequency).
    */
   public static ContinuousTransferFunction createHighPassFilterTransferFunction(double breakFrequency)
   {
      return new ContinuousTransferFunction("High-Pass Filter", 1.0, new double[] {1.0, 0.0}, new double[] {1.0, breakFrequency});
   }

   /**
    * Creates a second-order notch filter transfer function:
    * G(s) = (s^2 + notchFrequency^2) / (s^2 + notchWidth*s + notchFrequency^2).
    */
   public static ContinuousTransferFunction createNotchFilterTransferFunction(double notchFrequency, double notchWidth)
   {
      double notchFrequencySquared = notchFrequency * notchFrequency;
      return new ContinuousTransferFunction("Notch Filter",
                                             1.0,
                                             new double[] {1.0, 0.0, notchFrequencySquared},
                                             new double[] {1.0, notchWidth, notchFrequencySquared});
   }

   public void setNumerator(double[] numerator)
   {
      this.numerator = Arrays.copyOf(numerator, numerator.length);
   }

   public void setDenominator(double[] denominator)
   {
      this.denominator = Arrays.copyOf(denominator, denominator.length);
   }

   public void setGain(double k)
   {
      this.k = k;
   }

   /**
    * Returns the backing numerator array directly, without copying. Do not store a reference to it or modify it;
    * it is the live internal state of this transfer function.
    */
   public double[] getNumeratorUnsafe()
   {
      return numerator;
   }

   /**
    * Returns the backing denominator array directly, without copying. Do not store a reference to it or modify it;
    * it is the live internal state of this transfer function.
    */
   public double[] getDenominatorUnsafe()
   {
      return denominator;
   }

   /**
    * Copies the numerator coefficients into {@code numeratorToPack}, which must already be sized to match. Garbage free.
    */
   public void getNumerator(double[] numeratorToPack)
   {
      System.arraycopy(numerator, 0, numeratorToPack, 0, numerator.length);
   }

   /**
    * Copies the denominator coefficients into {@code denominatorToPack}, which must already be sized to match. Garbage free.
    */
   public void getDenominator(double[] denominatorToPack)
   {
      System.arraycopy(denominator, 0, denominatorToPack, 0, denominator.length);
   }

   public double getGain()
   {
      return k;
   }

   public String getName()
   {
      return name;
   }

   public void setName(String newName)
   {
      name = newName;
   }
}
