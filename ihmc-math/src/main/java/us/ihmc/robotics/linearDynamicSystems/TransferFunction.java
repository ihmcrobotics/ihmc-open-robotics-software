package us.ihmc.robotics.linearDynamicSystems;

import us.ihmc.robotics.dataStructures.ComplexNumber;
import us.ihmc.robotics.dataStructures.ComplexPolynomialTools;
import us.ihmc.robotics.math.trajectories.core.Polynomial;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialBasics;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialReadOnly;

/**
 * TransferFunction.
 * Important rep invariant: The numerator and denominator will be scaled so that the highest order in the denominator is always 1.0.
 * For example (30s + 15)/(3s + 6) will be scaled to (10s + 5)/(s + 2)
 */
public class TransferFunction
{
   private final PolynomialReadOnly numerator, denominator;

   public TransferFunction(double[] numerator, double[] denominator)
   {
      Polynomial numeratorPolynomial = new Polynomial(numerator);
      Polynomial denominatorPolynomial = new Polynomial(denominator);

      double scaleFactor = 1.0 / denominatorPolynomial.getCoefficients()[denominatorPolynomial.getOrder()];

      this.numerator = numeratorPolynomial.times(scaleFactor);
      this.denominator = denominatorPolynomial.times(scaleFactor);
   }

   public TransferFunction(PolynomialReadOnly numeratorPolynomial, PolynomialReadOnly denominatorPolynomial)
   {
      double scaleFactor = 1.0 / denominatorPolynomial.getCoefficients()[denominatorPolynomial.getOrder()];

      // creates and packs a copy, unlinking from the input polynomials.
      this.numerator = numeratorPolynomial.times(scaleFactor);
      this.denominator = denominatorPolynomial.times(scaleFactor);
   }

   public static TransferFunction constructSecondOrderTransferFunction(double zeroFreqMagnitude, double wn, double zeta)
   {
      double[] numerator = new double[] {zeroFreqMagnitude * wn * wn};
      double[] denominator = new double[] {wn * wn, 2.0 * zeta * wn, 1.0};

      return new TransferFunction(numerator, denominator);
   }

   public static TransferFunction constructZeroTransferFunction()
   {
      return new TransferFunction(new double[] {0.0}, new double[] {1.0});
   }

   public int getOrder()
   {
      return denominator.getOrder();
   }

   public ComplexNumber evaluate(ComplexNumber complexNumber)
   {
      ComplexNumber evaluateNumerator = ComplexPolynomialTools.evaluate(numerator, complexNumber);
      ComplexNumber evaluateDenominator = ComplexPolynomialTools.evaluate(denominator, complexNumber);

      ComplexNumber ret = evaluateNumerator.dividedBy(evaluateDenominator);

      return ret;
   }

   public double getMagnitude(double omega)
   {
      ComplexNumber jOmega = new ComplexNumber(0.0, omega);
      ComplexNumber evaluateNumerator = ComplexPolynomialTools.evaluate(numerator, jOmega);
      ComplexNumber evaluateDenominator = ComplexPolynomialTools.evaluate(denominator, jOmega);

      return evaluateNumerator.magnitude() / evaluateDenominator.magnitude();
   }

   public double getPhase(double omega)
   {
      ComplexNumber jOmega = new ComplexNumber(0.0, omega);
      ComplexNumber evaluateNumerator = ComplexPolynomialTools.evaluate(numerator, jOmega);
      ComplexNumber evaluateDenominator = ComplexPolynomialTools.evaluate(denominator, jOmega);

      return evaluateNumerator.angle() - evaluateDenominator.angle();
   }

   public double[] getMagnitude(double[] omega)
   {
      double[] magnitudes = new double[omega.length];
      for (int i = 0; i < omega.length; i++)
      {
         magnitudes[i] = getMagnitude(omega[i]);
      }

      return magnitudes;
   }

   public double[] getPhase(double[] omega)
   {
      double[] phases = new double[omega.length];
      for (int i = 0; i < omega.length; i++)
      {
         phases[i] = getPhase(omega[i]);
      }

      return phases;
   }

   public double[] getNumeratorCoefficients()
   {
      return numerator.getCoefficients();
   }

   public double[] getDenominatorCoefficients()
   {
      return denominator.getCoefficients();
   }

   public PolynomialReadOnly getNumeratorPolynomial()
   {
      return numerator;
   }

   public PolynomialReadOnly getDenominatorPolynomial()
   {
      return denominator;
   }

   /**
    * If this transfer function is a / b, this returns a / b + d, which is (a + d * b) / b
    *
    * @param d scalar parameter to add to this transfer function.
    * @return the modified transfer function including d
    */
   public TransferFunction plus(double d)
   {
      return new TransferFunction(numerator.plus((denominator.times(d))), denominator);
   }

   public TransferFunction times(double d)
   {
      PolynomialBasics newNumerator = numerator.times(d);
      if (newNumerator.equalsZero(1e-15))
      {
         return constructZeroTransferFunction();
      }

      return new TransferFunction(newNumerator, denominator);
   }

   /**
    * Computes the combined transfer function,
    * <p>
    * C = A + B, where A is (a / c) and B is (b / d), and A is this object.
    * </p>
    * <p>
    * via distribution, this is
    * </p>
    * <p>
    * C = (a * d + b * c) / (c * d)
    * </p>
    *
    * @param transferFunction B above
    * @return C above
    */
   public TransferFunction plus(TransferFunction transferFunction)
   {
      PolynomialReadOnly otherNumerator = transferFunction.getNumeratorPolynomial();
      PolynomialReadOnly otherDenominator = transferFunction.getDenominatorPolynomial();

      if (this.equalsZero())
      {
         return new TransferFunction(transferFunction.numerator, transferFunction.denominator);
      }
      else if (transferFunction.equalsZero())
      {
         return new TransferFunction(numerator, denominator);
      }
      else if (denominator.epsilonEquals(otherDenominator, 1e-7))
      {
         PolynomialReadOnly newDenominator = denominator;
         PolynomialBasics newNumerator = numerator.plus(otherNumerator);

         return new TransferFunction(newNumerator, newDenominator);
      }
      else
      {
         // a / c * b / d = (a * d + b * c) / (c * d)
         PolynomialReadOnly newNumerator = (numerator.times(otherDenominator)).plus(otherNumerator.times(denominator));
         PolynomialReadOnly newDenominator = denominator.times(otherDenominator);

         return new TransferFunction(newNumerator, newDenominator);
      }
   }

   /**
    * Returns whether this transfer function equals zero.
    */
   public boolean equalsZero()
   {
      return numerator.equalsZero(1e-15);
   }

   /**
    * Computes the multiplied transfer function,
    * <p>
    * C = A * B, where A is (a / c) and B is (b / d) and A is this object.
    * </p>
    * <p>
    * via multiplication, this is
    * </p>
    * <p>
    *    C = (a * b) / (c * d)
    * </p>
    *
    * @param transferFunction B above
    * @return C above
    */
   public TransferFunction times(TransferFunction transferFunction)
   {
      PolynomialReadOnly otherNumerator = transferFunction.getNumeratorPolynomial();
      PolynomialReadOnly otherDenominator = transferFunction.getDenominatorPolynomial();

      // First check if any cancellation of numerator and denominator:
      // @todo: Implement cancellation or root finding...
      // if (this.numerator.epsilonEquals(otherDenominator, 1e-10))
      // {
      // System.err.println("Cancellation!");
      // }
      //
      // if (this.denominator.epsilonEquals(otherNumerator, 1e-10))
      // {
      // System.err.println("Cancellation!");
      // }

      PolynomialBasics newNumerator = numerator.times(otherNumerator);
      PolynomialBasics newDenominator = denominator.times(otherDenominator);

      return new TransferFunction(newNumerator, newDenominator);
   }

   public boolean epsilonEquals(TransferFunction transferFunction, double epsilon)
   {
      // Since the rep invariant is to have the denominator start with 1.0, we should be able to compare the Polynomial coefficients directly.
      if (!this.numerator.epsilonEquals(transferFunction.numerator, epsilon))
         return false;
      if (!this.denominator.epsilonEquals(transferFunction.denominator, epsilon))
         return false;

      return true;
   }

   public String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("(");
      builder.append(this.numerator.toString());
      builder.append(")");
      builder.append("/(");
      builder.append(this.denominator.toString());
      builder.append(")");

      return builder.toString();
   }
}
