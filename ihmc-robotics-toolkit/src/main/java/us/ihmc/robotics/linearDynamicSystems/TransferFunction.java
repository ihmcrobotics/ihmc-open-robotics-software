package us.ihmc.robotics.linearDynamicSystems;

import us.ihmc.robotics.dataStructures.ComplexNumber;
import us.ihmc.robotics.dataStructures.Polynomial;

/**
 * TransferFunction.
 * Important rep invariant: The numerator and denominator will be scaled so that the highest order in the denominator is always 1.0.
 * For example (30s + 15)/(3s + 6) will be scaled to (10s + 5)/(s + 2)
 */
public class TransferFunction
{
   private final Polynomial numerator, denominator;

   public TransferFunction(double[] numerator, double[] denominator)
   {
      Polynomial numeratorPolynomial = new Polynomial(numerator);
      Polynomial denominatorPolynomial = new Polynomial(denominator);

      double scaleFactor = 1.0 / denominatorPolynomial.getCoefficients()[0];

      this.numerator = numeratorPolynomial.times(scaleFactor);
      this.denominator = denominatorPolynomial.times(scaleFactor);
   }

   public TransferFunction(Polynomial numeratorPolynomial, Polynomial denominatorPolynomial)
   {
      double scaleFactor = 1.0 / denominatorPolynomial.getCoefficients()[0];

      this.numerator = numeratorPolynomial.times(scaleFactor);
      this.denominator = denominatorPolynomial.times(scaleFactor);
   }

   public static TransferFunction constructSecondOrderTransferFunction(double zeroFreqMagnitude, double wn, double zeta)
   {
      double[] numerator = new double[] {zeroFreqMagnitude * wn * wn};
      double[] denominator = new double[] {1.0, 2.0 * zeta * wn, wn * wn};

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
      ComplexNumber evaluateNumerator = numerator.evaluate(complexNumber);
      ComplexNumber evaluateDenominator = denominator.evaluate(complexNumber);

      ComplexNumber ret = evaluateNumerator.dividedBy(evaluateDenominator);

      return ret;
   }


   public double getMagnitude(double omega)
   {
      ComplexNumber jOmega = new ComplexNumber(0.0, omega);
      ComplexNumber evaluateNumerator = numerator.evaluate(jOmega);
      ComplexNumber evaluateDenominator = denominator.evaluate(jOmega);

      return evaluateNumerator.magnitude() / evaluateDenominator.magnitude();
   }

   public double getPhase(double omega)
   {
      ComplexNumber jOmega = new ComplexNumber(0.0, omega);
      ComplexNumber evaluateNumerator = numerator.evaluate(jOmega);
      ComplexNumber evaluateDenominator = denominator.evaluate(jOmega);

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

   public Polynomial getNumeratorPolynomial()
   {
      return numerator;
   }

   public Polynomial getDenominatorPolynomial()
   {
      return denominator;
   }

   public TransferFunction plus(double d)
   {
      return new TransferFunction(denominator.times(d).plus(numerator), denominator);
   }

   public TransferFunction times(double d)
   {
      Polynomial newNumerator = numerator.times(d);
      if (newNumerator.equalsZero())
      {
         return constructZeroTransferFunction();
      }

      return new TransferFunction(newNumerator, denominator);
   }

   public TransferFunction plus(TransferFunction transferFunction)
   {
      Polynomial otherNumerator = transferFunction.getNumeratorPolynomial();
      Polynomial otherDenominator = transferFunction.getDenominatorPolynomial();

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
         Polynomial newDenominator = denominator;
         Polynomial newNumerator = numerator.plus(otherNumerator);

         return new TransferFunction(newNumerator, newDenominator);
      }
      else
      {
         Polynomial newDenominator = denominator.times(otherDenominator);
         Polynomial newNumerator = numerator.times(otherDenominator).plus(otherNumerator.times(denominator));

         return new TransferFunction(newNumerator, newDenominator);
      }
   }

   public boolean equalsZero()
   {
      return numerator.equalsZero();
   }

   public TransferFunction times(TransferFunction transferFunction)
   {
      Polynomial otherNumerator = transferFunction.getNumeratorPolynomial();
      Polynomial otherDenominator = transferFunction.getDenominatorPolynomial();

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


      Polynomial newNumerator = numerator.times(otherNumerator);
      Polynomial newDenominator = denominator.times(otherDenominator);

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
