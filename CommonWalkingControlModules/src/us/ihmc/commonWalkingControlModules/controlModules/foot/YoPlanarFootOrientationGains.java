package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.robotics.controllers.GainCalculator;
import us.ihmc.robotics.controllers.MatrixUpdater;
import us.ihmc.robotics.controllers.OrientationPIDGainsInterface;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

public class YoPlanarFootOrientationGains implements YoOrientationPIDGainsInterface
{
   private final DoubleYoVariable proportionalYGain;
   private final DoubleYoVariable derivativeYGain;
   private final DoubleYoVariable dampingRatioY;

   private final DoubleYoVariable maxDerivativeError;
   private final DoubleYoVariable maxProportionalError;

   private final DoubleYoVariable maximumFeedback;
   private final DoubleYoVariable maximumFeedbackRate;

   public YoPlanarFootOrientationGains(String suffix, YoVariableRegistry registry)
   {
      proportionalYGain = new DoubleYoVariable("kpYAngular" + suffix, registry);
      derivativeYGain = new DoubleYoVariable("kdYAngular" + suffix, registry);
      dampingRatioY = new DoubleYoVariable("zetaYAngular" + suffix, registry);

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
      proportionalYGain.set(0.0);
      derivativeYGain.set(0.0);
      maximumFeedback.set(Double.POSITIVE_INFINITY);
      maximumFeedbackRate.set(Double.POSITIVE_INFINITY);
      maxDerivativeError.set(Double.POSITIVE_INFINITY);
      maxProportionalError.set(Double.POSITIVE_INFINITY);
   }

   @Override
   public Matrix3DReadOnly createProportionalGainMatrix()
   {
      Matrix3D proportionalGainMatrix = new Matrix3D();

      proportionalYGain.addVariableChangedListener(new MatrixUpdater(1, 1, proportionalGainMatrix));
      proportionalYGain.notifyVariableChangedListeners();

      return proportionalGainMatrix;
   }

   @Override
   public Matrix3DReadOnly createDerivativeGainMatrix()
   {
      Matrix3D derivativeGainMatrix = new Matrix3D();

      derivativeYGain.addVariableChangedListener(new MatrixUpdater(1, 1, derivativeGainMatrix));
      derivativeYGain.notifyVariableChangedListeners();

      return derivativeGainMatrix;
   }

   @Override
   public Matrix3DReadOnly createIntegralGainMatrix()
   {
      return new Matrix3D();
   }

   public void createDerivativeGainUpdater(boolean updateNow)
   {
      VariableChangedListener kdYUpdater = new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            derivativeYGain.set(GainCalculator.computeDerivativeGain(proportionalYGain.getDoubleValue(), dampingRatioY.getDoubleValue()));
         }
      };

      proportionalYGain.addVariableChangedListener(kdYUpdater);
      dampingRatioY.addVariableChangedListener(kdYUpdater);

      if (updateNow)
         kdYUpdater.variableChanged(null);
   }

   @Override
   public void setProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      proportionalYGain.set(proportionalGainY);
   }

   public void setProportionalGains(double proportionalGainY)
   {
      setProportionalGains(0.0, proportionalGainY, 0.0);
   }

   @Override
   public void setDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      derivativeYGain.set(derivativeGainY);
   }

   public void setDerivativeGains(double derivativeGainY)
   {
      setDerivativeGains(0.0, derivativeGainY, 0.0);
   }

   public void setDampingRatio(double dampingRatio)
   {
      dampingRatioY.set(dampingRatio);
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
      tempPropotionalGains[0] = proportionalYGain.getDoubleValue();

      return tempPropotionalGains;
   }

   private double[] tempDerivativeGains = new double[1];

   @Override
   public double[] getDerivativeGains()
   {
      tempDerivativeGains[0] = derivativeYGain.getDoubleValue();

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
