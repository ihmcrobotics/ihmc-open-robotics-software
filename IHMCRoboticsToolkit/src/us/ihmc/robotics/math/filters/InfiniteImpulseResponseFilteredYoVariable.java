package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.dataStructures.ComplexNumber;
import us.ihmc.robotics.dataStructures.Polynomial;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2006</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class InfiniteImpulseResponseFilteredYoVariable extends DoubleYoVariable
{
   private final boolean DEBUG = true;

   private final int numPoles, numZeros;

   private final double[] denominatorCoefficients, numeratorCoefficients;

   // @todo: YoVariablize these states to make rewindable.
   private final double[] previousX, previousY;

   public InfiniteImpulseResponseFilteredYoVariable(String name, int numPoles, int numZeros, YoVariableRegistry registry)
   {
      super(name, registry);

      this.numPoles = numPoles;
      this.numZeros = numZeros;

      this.numeratorCoefficients = new double[numZeros + 1];
      this.denominatorCoefficients = new double[numPoles + 1];

      this.previousX = new double[numZeros];
      this.previousY = new double[numPoles];
   }

   public void setCoefficients(double[] numeratorCoefficients, double[] denominatorCoefficients)
   {
      if (numeratorCoefficients.length != this.numeratorCoefficients.length)
         throw new RuntimeException("Wrong number of coefficients in b! numeratorCoefficients.length = " + numeratorCoefficients.length
               + ", this.numeratorCoefficients.length = " + this.numeratorCoefficients.length);
      if (denominatorCoefficients.length != this.denominatorCoefficients.length)
         throw new RuntimeException("Wrong number of coefficients in a! denominatorCoefficients.length = " + denominatorCoefficients.length
               + ", this.denominatorCoefficients = " + this.denominatorCoefficients.length);

      for (int i = 0; i < numeratorCoefficients.length; i++)
      {
         this.numeratorCoefficients[i] = numeratorCoefficients[i];
      }

      for (int i = 0; i < denominatorCoefficients.length; i++)
      {
         this.denominatorCoefficients[i] = denominatorCoefficients[i];
      }

      if (DEBUG)
      {
         System.out.println("numeratorCoefficients = " + coefficientsToString(numeratorCoefficients));
         System.out.println("denominatorCoefficients = " + coefficientsToString(denominatorCoefficients));
      }
   }

   private String coefficientsToString(double[] coefficients)
   {
      String ret = "[";

      for (int i = 0; i < coefficients.length; i++)
      {
         ret = ret + coefficients[i] + " ";
      }

      ret = ret + "]";

      return ret;
   }

   public void setPolesAndZeros(double[] realPoles, ComplexNumber[] complexPairPoles, double[] realZeros, ComplexNumber[] complexPairZeros)
   {
      setPolesAndZeros(1.0, realPoles, complexPairPoles, realZeros, complexPairZeros);

      double numeratorSum = 0.0, denominatorSum = 0.0;

      for (double numeratorCoefficient : numeratorCoefficients)
      {
         numeratorSum += numeratorCoefficient;
      }

      for (double denominatorCoefficient : denominatorCoefficients)
      {
         denominatorSum += denominatorCoefficient;
      }

      double evalAtZEqualsOne = numeratorSum / denominatorSum;

      for (int i = 0; i < numeratorCoefficients.length; i++)
      {
         numeratorCoefficients[i] = numeratorCoefficients[i] / evalAtZEqualsOne;
      }
   }

   public void setPolesAndZeros(double dcScale, double[] realPoles, ComplexNumber[] complexPairPoles, double[] realZeros, ComplexNumber[] complexPairZeros)
   {
      Polynomial numeratorPolynomial = Polynomial.constructFromScaleFactorAndRoots(dcScale, realZeros, complexPairZeros);
      Polynomial denominatorPolynomial = Polynomial.constructFromScaleFactorAndRoots(1.0, realPoles, complexPairPoles);

      if (DEBUG)
      {
         System.out.println("denominatorPolynomial = " + denominatorPolynomial);
         System.out.println("numeratorPolynomial = " + numeratorPolynomial);
      }

      this.setCoefficients(numeratorPolynomial.getCoefficients(), denominatorPolynomial.getCoefficients());
   }

   public void update(double currentX)
   {
      //    this.val = 0.95 * previousY[0] + (1.0 - 0.95) * currentX;
      //    previousY[0] = this.val;
      //
      //    if (true) return;

      // @todo: Verify that this logic is correct. It might all be backwards order...
      // @todo: Add some unit tests to verify its ok.

      @SuppressWarnings("unused")
      double currentY = this.getDoubleValue();

      double newVal = 0.0;

      for (int i = 0; i < numPoles; i++)
      {
         newVal -= denominatorCoefficients[i + 1] * previousY[i];
      }

      for (int i = 0; i < numZeros; i++)
      {
         newVal += numeratorCoefficients[i + 1] * previousX[i];
      }

      newVal += numeratorCoefficients[0] * currentX;

      this.set(newVal);

      for (int i = numPoles - 1; i > 0; i--)
      {
         previousY[i] = previousY[i - 1];
      }

      if (previousY.length > 0)
         previousY[0] = newVal; // currentY;

      for (int i = numZeros - 1; i > 0; i--)
      {
         previousX[i] = previousX[i - 1];
      }

      if (previousX.length > 0)
         previousX[0] = currentX;

      throw new RuntimeException("This needs to be tested before trusting it. Check out the main and write a JUnit test case!");

   }
}
