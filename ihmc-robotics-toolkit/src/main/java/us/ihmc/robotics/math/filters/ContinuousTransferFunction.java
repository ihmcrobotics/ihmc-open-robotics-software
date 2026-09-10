package us.ihmc.robotics.math.filters;

/**
 * This class holds the representation of a continuous transfer function of the form:
 * <pre>
 * G(s) = k * (b_0*s^m + b_1*s^{m-1} + ... + b_{m-1}*s + b_m)/(a_0*s^n + a_1*s^{n-1} + ... + a_{n-1}*s + a_n)
 * </pre>
 * where numerator[] = [b_0, b_1, ..., b_{m-1}, b_m] and denominator[] = [a_0, a_1, ..., a_{n-1}, a_n]. This class can
 * be passed into the TransferFunctionDiscretizer to create a discrete representation of that transfer function
 * in the YoFilteredDouble class.
 * In addition, two ContinuousTransferFunctions can be cascaded together to produce a new transfer function of the form
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
      this.numerator = numerator;
      this.denominator = denominator;
   }

   public ContinuousTransferFunction(ContinuousTransferFunction[] severalContinuousTransferFunctions)
   {
      this("", severalContinuousTransferFunctions);
   }

   public ContinuousTransferFunction(String name, ContinuousTransferFunction[] severalContinuousTransferFunctions)
   {
      this.name = name;

      int numOfTransferFunctionsToCascade = severalContinuousTransferFunctions.length;
      ContinuousTransferFunction tf_total;

      // Initialize total transfer function with first transfer function.
      tf_total = severalContinuousTransferFunctions[0];

      for (int i = 1; i < numOfTransferFunctionsToCascade; i++)
      {
         tf_total = CascadeTwoTransferFunctions(tf_total, severalContinuousTransferFunctions[i]);
      }

      // Set private variables to total transfer function variables;
      this.k = tf_total.getGain();
      this.numerator = tf_total.getNumerator();
      this.denominator = tf_total.getDenominator();
   }

   /**
    * Method cascades two transfer functions, H1(s) and H2(s) and returns a new transfer function H3(s).
    * <pre>
    *                            N1(s)      N2(s)        N3(s)
    * H3(s) = H1(s)*H2(s) = k1 * ----- k2 * ----- = k3 * -----
    *                            D1(s)      D2(s)        D3(s)
    * </pre>
    *
    * @param H1 - Transfer Function 1
    * @param H2 - Transfer Function 2
    * @return H3 - Combined Transfer Function of 1 and 2 (H3(s) = H1(s)*H2(s))
    */
   private ContinuousTransferFunction CascadeTwoTransferFunctions(ContinuousTransferFunction H1, ContinuousTransferFunction H2)
   {
      ContinuousTransferFunction H3 = new ContinuousTransferFunction();

      H3.setGain(H1.getGain() * H2.getGain());
      H3.setNumerator(cascadePolynomials(H1.getNumerator(), H2.getNumerator()));
      H3.setDenominator(cascadePolynomials(H1.getDenominator(), H2.getDenominator()));

      return H3;
   }

   /**
    * Method combines two polynomials, poly1(x) and poly2(x), into poly3(s).
    *
    * @param poly1
    * @param poly2
    * @return poly3 = poly1*poly2
    */
   private static double[] cascadePolynomials(double[] poly1, double[] poly2)
   {
      // Initialize new polynomial to new size.
      double[] poly3 = new double[poly1.length + poly2.length - 1];

      // Cycle through poly1 and poly2 to polynomials.
      for (int i = 0; i < poly1.length; i++)
      {
         for (int j = 0; j < poly2.length; j++)
         {
            poly3[i + j] += poly1[i] * poly2[j];
         }
      }

      return poly3;
   }

   public void setNumerator(double[] numerator)
   {
      this.numerator = numerator;
   }

   public void setDenominator(double[] denominator)
   {
      this.denominator = denominator;
   }

   public void setGain(double k)
   {
      this.k = k;
   }

   public double[] getNumerator()
   {
      return numerator;
   }

   public double[] getDenominator()
   {
      return denominator;
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
