package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.robotics.controllers.*;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

import javax.vecmath.Matrix3d;

public class YoPlanarFootOrientationGains implements YoOrientationPIDGainsInterface
{
   private final DoubleYoVariable proportionalXGain;
   private final DoubleYoVariable derivativeXGain;
   private final DoubleYoVariable dampingRatioX;

   private final DoubleYoVariable maxDerivativeError;
   private final DoubleYoVariable maxProportionalError;

   private final DoubleYoVariable maximumFeedback;
   private final DoubleYoVariable maximumFeedbackRate;

   public YoPlanarFootOrientationGains(String suffix, YoVariableRegistry registry)
   {
      proportionalXGain = new DoubleYoVariable("kpXAngular" + suffix, registry);
      derivativeXGain = new DoubleYoVariable("kdXAngular" + suffix, registry);
      dampingRatioX = new DoubleYoVariable("zetaXAngular" + suffix, registry);

      maximumFeedback = new DoubleYoVariable("maximumAngularFeedback" + suffix, registry);
      maximumFeedbackRate = new DoubleYoVariable("maximumAngularFeedbackRate" + suffix, registry);

      maxDerivativeError = new DoubleYoVariable("maximumAngularDerivativeError" + suffix, registry);
      maxProportionalError = new DoubleYoVariable("maximumAngularProportionalError" + suffix, registry);

      maximumFeedback.set(Double.POSITIVE_INFINITY);
      maximumFeedbackRate.set(Double.POSITIVE_INFINITY);

      maxDerivativeError.set(Double.POSITIVE_INFINITY);
      maxProportionalError.set(Double.POSITIVE_INFINITY);
   }

   public void reset()
   {
      proportionalXGain.set(0.0);
      derivativeXGain.set(0.0);
      maximumFeedback.set(Double.POSITIVE_INFINITY);
      maximumFeedbackRate.set(Double.POSITIVE_INFINITY);
      maxDerivativeError.set(Double.POSITIVE_INFINITY);
      maxProportionalError.set(Double.POSITIVE_INFINITY);
   }

   @Override
   public Matrix3d createProportionalGainMatrix()
   {
      Matrix3d proportionalGainMatrix = new Matrix3d();

      proportionalXGain.addVariableChangedListener(new MatrixUpdater(0, 0, proportionalGainMatrix));
      proportionalXGain.notifyVariableChangedListeners();

      return proportionalGainMatrix;
   }

   @Override
   public Matrix3d createDerivativeGainMatrix()
   {
      Matrix3d derivativeGainMatrix = new Matrix3d();

      derivativeXGain.addVariableChangedListener(new MatrixUpdater(0, 0, derivativeGainMatrix));
      derivativeXGain.notifyVariableChangedListeners();

      return derivativeGainMatrix;
   }

   @Override
   public Matrix3d createIntegralGainMatrix()
   {
      return new Matrix3d();
   }

   public void createDerivativeGainUpdater(boolean updateNow)
   {
      VariableChangedListener kdXUpdater = new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            derivativeXGain.set(GainCalculator.computeDerivativeGain(proportionalXGain.getDoubleValue(), dampingRatioX.getDoubleValue()));
         }
      };

      proportionalXGain.addVariableChangedListener(kdXUpdater);
      dampingRatioX.addVariableChangedListener(kdXUpdater);

      if (updateNow)
         kdXUpdater.variableChanged(null);
   }

   @Override
   public void setProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      proportionalXGain.set(proportionalGainX);
   }

   public void setProportionalGains(double proportionalGainX)
   {
      setProportionalGains(proportionalGainX, 0.0, 0.0);
   }

   @Override
   public void setDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      derivativeXGain.set(derivativeGainX);
   }

   public void setDerivativeGains(double derivativeGainX)
   {
      setDerivativeGains(derivativeGainX, 0.0, 0.0);
   }

   public void setDampingRatio(double dampingRatio)
   {
      dampingRatioX.set(dampingRatio);
   }

   @Override
   public void setIntegralGains(double integralGainX, double integralGainY, double integralGainZ, double maxIntegralError)
   {
   }

   @Override
   public void setProportionalGains(double[] proportionalGains)
   {
      setProportionalGains(proportionalGains[0], 0.0, 0.0);
   }

   @Override
   public void setDerivativeGains(double[] derivativeGains)
   {
      setDerivativeGains(derivativeGains[0], 0.0, 0.0);
   }

   @Override
   public void setIntegralGains(double[] integralGains, double maxIntegralError)
   {
      setIntegralGains(integralGains[0], 0.0, 0.0, maxIntegralError);
   }

   public void setMaximumFeedback(double maxFeedback)
   {
      maximumFeedback.set(maxFeedback);
   }

   public void setMaximumFeedbackRate(double maxFeedbackRate)
   {
      maximumFeedbackRate.set(maxFeedbackRate);
   }

   @Override
   public void setMaxFeedbackAndFeedbackRate(double maxFeedback, double maxFeedbackRate)
   {
      maximumFeedback.set(maxFeedback);
      maximumFeedbackRate.set(maxFeedbackRate);
   }

   @Override
   public void setMaxDerivativeError(double maxDerivativeError)
   {
      this.maxDerivativeError.set(maxDerivativeError);
   }

   @Override
   public void setMaxProportionalError(double maxProportionalError)
   {
      this.maxProportionalError.set(maxProportionalError);
   }

   @Override
   public void set(OrientationPIDGainsInterface gains)
   {
      setProportionalGains(gains.getProportionalGains());
      setDerivativeGains(gains.getDerivativeGains());
      setIntegralGains(gains.getIntegralGains(), gains.getMaximumIntegralError());
      setMaxFeedbackAndFeedbackRate(gains.getMaximumFeedback(), gains.getMaximumFeedbackRate());
      setMaxDerivativeError(gains.getMaximumDerivativeError());
      setMaxProportionalError(gains.getMaximumProportionalError());
   }

   @Override
   public DoubleYoVariable getYoMaximumFeedback()
   {
      return maximumFeedback;
   }

   @Override
   public DoubleYoVariable getYoMaximumFeedbackRate()
   {
      return maximumFeedbackRate;
   }

   @Override
   public DoubleYoVariable getYoMaximumDerivativeError()
   {
      return maxDerivativeError;
   }

   @Override
   public DoubleYoVariable getYoMaximumProportionalError()
   {
      return maxProportionalError;
   }

   private double[] tempPropotionalGains = new double[1];

   @Override
   public double[] getProportionalGains()
   {
      tempPropotionalGains[0] = proportionalXGain.getDoubleValue();

      return tempPropotionalGains;
   }

   private double[] tempDerivativeGains = new double[1];

   @Override
   public double[] getDerivativeGains()
   {
      tempDerivativeGains[0] = derivativeXGain.getDoubleValue();

      return tempDerivativeGains;
   }

   private double[] tempIntegralGains = new double[1];

   @Override
   public double[] getIntegralGains()
   {
      return tempIntegralGains;
   }

   @Override
   public double getMaximumIntegralError()
   {
      return 0.0;
   }

   @Override
   public double getMaximumFeedback()
   {
      return maximumFeedback.getDoubleValue();
   }

   @Override
   public double getMaximumFeedbackRate()
   {
      return maximumFeedbackRate.getDoubleValue();
   }

   @Override
   public double getMaximumDerivativeError()
   {
      return maxDerivativeError.getDoubleValue();
   }

   @Override
   public double getMaximumProportionalError()
   {
      return maxProportionalError.getDoubleValue();
   }

}
