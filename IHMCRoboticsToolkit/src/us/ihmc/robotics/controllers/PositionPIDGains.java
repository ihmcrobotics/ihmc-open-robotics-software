package us.ihmc.robotics.controllers;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.robotics.controllers.pidGains.PositionPIDGainsInterface;

public class PositionPIDGains implements PositionPIDGainsInterface
{
   private double[] positionProportionalGains = new double[3];
   private double[] positionDerivativeGains = new double[3];
   private double[] positionIntegralGains = new double[3];

   private double positionMaxIntegralError = 0.0;
   private double positionMaxDerivativeError = Double.POSITIVE_INFINITY;
   private double positionMaxProportionalError = Double.POSITIVE_INFINITY;

   private double positionMaximumFeedback = Double.POSITIVE_INFINITY;
   private double positionMaximumFeedbackRate = Double.POSITIVE_INFINITY;

   private TangentialDampingGains tangentialDampingGains;

   private final Matrix3D proportionalGainMatrix = new Matrix3D();
   private final Matrix3D derivativeGainMatrix = new Matrix3D();
   private final Matrix3D integralGainMatrix = new Matrix3D();

   public void set(PositionPIDGainsInterface gains)
   {
      for (int i = 0; i < 3; i++)
      {
         positionProportionalGains[i] = gains.getProportionalGains()[i];
         positionDerivativeGains[i] = gains.getDerivativeGains()[i];
         positionIntegralGains[i] = gains.getIntegralGains()[i];
      }

      positionMaxIntegralError = gains.getMaximumIntegralError();
      positionMaxDerivativeError = gains.getMaximumDerivativeError();
      positionMaxProportionalError = gains.getMaximumProportionalError();

      positionMaximumFeedback = gains.getMaximumFeedback();
      positionMaximumFeedbackRate = gains.getMaximumFeedbackRate();

      tangentialDampingGains = gains.getTangentialDampingGains();

      updateGainMatrices();
   }

   public void setGains(double proportionalGain, double derivativeGain)
   {
      setGains(proportionalGain, derivativeGain, 0.0, 0.0);
   }

   public void setGains(double proportionalGain, double derivativeGain, double integralGain, double maxIntegralError)
   {
      setGains(proportionalGain, derivativeGain, integralGain, maxIntegralError, 1.0, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
   }

   public void setGains(double proportionalGain, double derivativeGain, double integralGain, double maxIntegralError, double kdReductionRatio,
         double parallelDampingDeadband, double positionErrorForMinimumKd)
   {
      setProportionalGains(proportionalGain, proportionalGain, proportionalGain);
      setDerivativeGains(derivativeGain, derivativeGain, derivativeGain);
      setIntegralGains(integralGain, integralGain, integralGain, maxIntegralError);
      setTangentialDampingGains(kdReductionRatio, parallelDampingDeadband, positionErrorForMinimumKd);
   }

   @Override
   public void setProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      this.positionProportionalGains[0] = proportionalGainX;
      this.positionProportionalGains[1] = proportionalGainY;
      this.positionProportionalGains[2] = proportionalGainZ;
      updateGainMatrices();
   }

   @Override
   public void setDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      this.positionDerivativeGains[0] = derivativeGainX;
      this.positionDerivativeGains[1] = derivativeGainY;
      this.positionDerivativeGains[2] = derivativeGainZ;
      updateGainMatrices();
   }

   @Override
   public void setIntegralGains(double integralGainX, double integralGainY, double integralGainZ, double maxIntegralError)
   {
      this.positionIntegralGains[0] = integralGainX;
      this.positionIntegralGains[1] = integralGainY;
      this.positionIntegralGains[2] = integralGainZ;
      this.positionMaxIntegralError = maxIntegralError;
      updateGainMatrices();
   }

   public void setTangentialDampingGains(double kdReductionRatio, double parallelDampingDeadband, double positionErrorForMinimumKd)
   {
      if (tangentialDampingGains != null)
         tangentialDampingGains.set(kdReductionRatio, parallelDampingDeadband, positionErrorForMinimumKd);
   }

   @Override
   public void setMaxFeedbackAndFeedbackRate(double maxFeedback, double maxFeedbackRate)
   {
      positionMaximumFeedback = maxFeedback;
      positionMaximumFeedbackRate = maxFeedbackRate;
   }

   @Override
   public void setMaxDerivativeError(double maxDerivativeError)
   {
      positionMaxDerivativeError = maxDerivativeError;
   }

   @Override
   public void setMaxProportionalError(double maxProportionalError)
   {
      positionMaxProportionalError = maxProportionalError;
   }

   @Override
   public double[] getProportionalGains()
   {
      return positionProportionalGains;
   }

   @Override
   public double[] getDerivativeGains()
   {
      return positionDerivativeGains;
   }

   @Override
   public double[] getIntegralGains()
   {
      return positionIntegralGains;
   }

   @Override
   public double getMaximumIntegralError()
   {
      return positionMaxIntegralError;
   }

   @Override
   public double getMaximumFeedback()
   {
      return positionMaximumFeedback;
   }

   @Override
   public double getMaximumFeedbackRate()
   {
      return positionMaximumFeedbackRate;
   }

   @Override
   public double getMaximumDerivativeError()
   {
      return positionMaxDerivativeError;
   }

   @Override
   public double getMaximumProportionalError()
   {
      return positionMaxProportionalError;
   }

   @Override
   public TangentialDampingGains getTangentialDampingGains()
   {
      return tangentialDampingGains;
   }

   private void updateGainMatrices()
   {
      proportionalGainMatrix.fill(0.0);
      derivativeGainMatrix.fill(0.0);
      integralGainMatrix.fill(0.0);
      for (int i = 0; i < 3; i++)
      {
         proportionalGainMatrix.setElement(i, i, positionProportionalGains[i]);
         derivativeGainMatrix.setElement(i, i, positionDerivativeGains[i]);
         integralGainMatrix.setElement(i, i, positionIntegralGains[i]);
      }
   }

   @Override
   public Matrix3DReadOnly getProportionalGainMatrix()
   {
      return proportionalGainMatrix;
   }

   @Override
   public Matrix3DReadOnly getDerivativeGainMatrix()
   {
      return derivativeGainMatrix;
   }

   @Override
   public Matrix3DReadOnly getIntegralGainMatrix()
   {
      return integralGainMatrix;
   }
}
