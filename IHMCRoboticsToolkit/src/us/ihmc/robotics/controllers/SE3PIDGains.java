package us.ihmc.robotics.controllers;


/**
 * @author twan
 *         Date: 6/4/13
 */
public class SE3PIDGains
{
   private double[] positionProportionalGains = new double[3];
   private double[] positionDerivativeGains = new double[3];
   private double[] positionIntegralGains = new double[3];
   private double positionMaxIntegralError = 0.0;

   private double[] orientationProportionalGains = new double[3];
   private double[] orientationDerivativeGains = new double[3];
   private double[] orientationIntegralGains = new double[3];
   private double orientationMaxIntegralError = 0.0;

   private double positionMaximumAcceleration = Double.POSITIVE_INFINITY;
   private double positionMaximumJerk = Double.POSITIVE_INFINITY;
   private double orientationMaximumAcceleration = Double.POSITIVE_INFINITY;
   private double orientationMaximumJerk = Double.POSITIVE_INFINITY;

   public void set(double kpPosition, double zetaPosition, double kPOrientation, double zetaOrientation)
   {
      set(kpPosition, zetaPosition, 0.0, 0.0, kPOrientation, zetaOrientation, 0.0, 0.0);
   }

   public void set(double kpPosition, double zetaPosition, double kiPosition, double maxIntegralPosError, double kpOrientation, double zetaOrientation, double kiOrientation, double maxIntegralOriError)
   {
      double kdPosition = GainCalculator.computeDerivativeGain(kpPosition, zetaPosition);
      setPositionGains(kpPosition, kdPosition, kiPosition, maxIntegralPosError);

      double kdOrientation = GainCalculator.computeDerivativeGain(kpOrientation, zetaOrientation);
      setOrientationGains(kpOrientation, kdOrientation, kiOrientation, maxIntegralOriError);
   }

   public void setPositionGains(double proportionalGain, double derivativeGain)
   {
      setPositionGains(proportionalGain, derivativeGain, 0.0, 0.0);
   }

   public void setPositionGains(double proportionalGain, double derivativeGain, double integralGain, double maxIntegralError)
   {
      setPositionProportionalGains(proportionalGain, proportionalGain, proportionalGain);
      setPositionDerivativeGains(derivativeGain, derivativeGain, derivativeGain);
      setPositionIntegralGains(integralGain, integralGain, integralGain, maxIntegralError);
   }

   public void setPositionProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      this.positionProportionalGains[0] = proportionalGainX;
      this.positionProportionalGains[1] = proportionalGainY;
      this.positionProportionalGains[2] = proportionalGainZ;
   }

   public void setPositionDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      this.positionDerivativeGains[0] = derivativeGainX;
      this.positionDerivativeGains[1] = derivativeGainY;
      this.positionDerivativeGains[2] = derivativeGainZ;
   }

   public void setPositionIntegralGains(double integralGainX, double integralGainY, double integralGainZ, double maxIntegralError)
   {
      this.positionIntegralGains[0] = integralGainX;
      this.positionIntegralGains[1] = integralGainY;
      this.positionIntegralGains[2] = integralGainZ;
      this.positionMaxIntegralError = maxIntegralError;
   }

   public void setOrientationGains(double proportionalGain, double derivativeGain)
   {
      setOrientationGains(proportionalGain, derivativeGain, 0.0, 0.0);
   }

   public void setOrientationGains(double proportionalGain, double derivativeGain, double integralGain, double maxIntegralError)
   {
      setOrientationProportionalGains(proportionalGain, proportionalGain, proportionalGain);
      setOrientationDerivativeGains(derivativeGain, derivativeGain, derivativeGain);
      setOrientationIntegralGains(integralGain, integralGain, integralGain, maxIntegralError);
   }

   public void setOrientationProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      this.orientationProportionalGains[0] = proportionalGainX;
      this.orientationProportionalGains[1] = proportionalGainY;
      this.orientationProportionalGains[2] = proportionalGainZ;
   }

   public void setOrientationDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      this.orientationDerivativeGains[0] = derivativeGainX;
      this.orientationDerivativeGains[1] = derivativeGainY;
      this.orientationDerivativeGains[2] = derivativeGainZ;
   }

   public void setOrientationIntegralGains(double integralGainX, double integralGainY, double integralGainZ, double maxIntegralError)
   {
      this.orientationIntegralGains[0] = integralGainX;
      this.orientationIntegralGains[1] = integralGainY;
      this.orientationIntegralGains[2] = integralGainZ;
      this.orientationMaxIntegralError = maxIntegralError;
   }

   public void setMaximumAccelerationAndJerk(double maxAcceleration, double maxJerk)
   {
      positionMaximumAcceleration = maxAcceleration;
      positionMaximumJerk = maxJerk;
      
      orientationMaximumAcceleration = maxAcceleration;
      orientationMaximumJerk = maxJerk;
   }

   public void setPositionMaximumAccelerationAndJerk(double maxAcceleration, double maxJerk)
   {
      positionMaximumAcceleration = maxAcceleration;
      positionMaximumJerk = maxJerk;
   }

   public void setOrientationMaximumAccelerationAndJerk(double maxAcceleration, double maxJerk)
   {
      orientationMaximumAcceleration = maxAcceleration;
      orientationMaximumJerk = maxJerk;
   }

   public double[] getPositionProportionalGains()
   {
      return positionProportionalGains;
   }

   public double[] getPositionDerivativeGains()
   {
      return positionDerivativeGains;
   }

   public double[] getPositionIntegralGains()
   {
      return positionIntegralGains;
   }

   public double getPositionMaxIntegralError()
   {
      return positionMaxIntegralError;
   }

   public double[] getOrientationProportionalGains()
   {
      return orientationProportionalGains;
   }

   public double[] getOrientationDerivativeGains()
   {
      return orientationDerivativeGains;
   }

   public double[] getOrientationIntegralGains()
   {
      return orientationIntegralGains;
   }

   public double getOrientationMaxIntegralError()
   {
      return orientationMaxIntegralError;
   }

   public double getPositionMaximumAcceleration()
   {
      return positionMaximumAcceleration;
   }

   public double getPositionMaximumJerk()
   {
      return positionMaximumJerk;
   }

   public double getOrientationMaximumAcceleration()
   {
      return orientationMaximumAcceleration;
   }

   public double getOrientationMaximumJerk()
   {
      return orientationMaximumJerk;
   }

   @Override
   public String toString()
   {
      String positionProportional = "kpPosX: " + positionProportionalGains[0] + ", kpPosY: " + positionProportionalGains[1] + ", kpPosZ: " + positionProportionalGains[2];
      String positionDerivative = "kdPosX: " + positionDerivativeGains[0] + ", kdPosY: " + positionDerivativeGains[1] + ", kdPosZ: " + positionDerivativeGains[2];
      String positionIntegral = "kiPosX: " + positionIntegralGains[0] + ", kiPosY: " + positionIntegralGains[1] + ", kiPosZ: " + positionIntegralGains[2] + ", maxIntegralPosError: " + positionMaxIntegralError;
      String orientationProportional = "kpOriX: " + orientationProportionalGains[0] + ", kpOriY: " + orientationProportionalGains[1] + ", kpOriZ: " + orientationProportionalGains[2];
      String orientationDerivative = "kdOriX: " + orientationDerivativeGains[0] + ", kdOriY: " + orientationDerivativeGains[1] + ", kdOriZ: " + orientationDerivativeGains[2];
      String orientationIntegral = "kiOriX: " + orientationIntegralGains[0] + ", kiOriY: " + orientationIntegralGains[1] + ", kiOriZ: " + orientationIntegralGains[2] + ", maxIntegralOriError: " + orientationMaxIntegralError;

      return positionProportional + ", " + positionDerivative + ", " + positionIntegral + ", " + orientationProportional + ", " + orientationDerivative + ", " + orientationIntegral;
   }
}
