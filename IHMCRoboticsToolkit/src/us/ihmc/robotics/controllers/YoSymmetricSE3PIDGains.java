package us.ihmc.robotics.controllers;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoSymmetricSE3PIDGains implements YoSE3PIDGainsInterface, YoPositionPIDGainsInterface, YoOrientationPIDGainsInterface
{
   private final YoDouble proportionalGain;
   private final YoDouble derivativeGain;
   private final YoDouble dampingRatio;
   private final YoDouble integralGain;

   private final YoDouble maxIntegralError;
   private final YoDouble maxDerivativeError;
   private final YoDouble maxProportionalError;

   private final YoDouble maximumFeedback;
   private final YoDouble maximumFeedbackRate;

   private final YoTangentialDampingGains tangentialDampingGains;

   public YoSymmetricSE3PIDGains(String suffix, YoVariableRegistry registry)
   {
      proportionalGain = new YoDouble("kp" + suffix, registry);
      derivativeGain = new YoDouble("kd" + suffix, registry);
      dampingRatio = new YoDouble("zeta" + suffix, registry);
      integralGain = new YoDouble("ki" + suffix, registry);

      maxIntegralError = new YoDouble("maxIntegralError" + suffix, registry);
      maxDerivativeError = new YoDouble("maxDerivativeError" + suffix, registry);
      maxProportionalError = new YoDouble("maxProportionalError" + suffix, registry);

      maximumFeedback = new YoDouble("maximumFeedback" + suffix, registry);
      maximumFeedbackRate = new YoDouble("maximumFeedbackRate" + suffix, registry);

      tangentialDampingGains = new YoTangentialDampingGains(suffix, registry);

      maximumFeedback.set(Double.POSITIVE_INFINITY);
      maximumFeedbackRate.set(Double.POSITIVE_INFINITY);
      maxDerivativeError.set(Double.POSITIVE_INFINITY);
      maxProportionalError.set(Double.POSITIVE_INFINITY);
   }

   @Override
   public void reset()
   {
      proportionalGain.set(0.0);
      derivativeGain.set(0.0);
      dampingRatio.set(0.0);
      integralGain.set(0.0);

      maxIntegralError.set(0.0);
      maxDerivativeError.set(Double.POSITIVE_INFINITY);
      maxProportionalError.set(Double.POSITIVE_INFINITY);

      maximumFeedback.set(Double.POSITIVE_INFINITY);
      maximumFeedbackRate.set(Double.POSITIVE_INFINITY);
   }

   @Override
   public YoPositionPIDGainsInterface getPositionGains()
   {
      return this;
   }

   @Override
   public YoOrientationPIDGainsInterface getOrientationGains()
   {
      return this;
   }

   public void setProportionalGain(double proportionalGain)
   {
      this.proportionalGain.set(proportionalGain);
   }

   public void setDerivativeGain(double derivativeGain)
   {
      this.derivativeGain.set(derivativeGain);
   }

   public void setDampingRatio(double dampingRatio)
   {
      this.dampingRatio.set(dampingRatio);
   }

   public void setIntegralGain(double integralGain)
   {
      this.integralGain.set(integralGain);
   }

   public void setMaximumIntegralError(double maxIntegralError)
   {
      this.maxIntegralError.set(maxIntegralError);
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
   public void set(PositionPIDGainsInterface gains)
   {
      setProportionalGains(gains.getProportionalGains());
      setDerivativeGains(gains.getDerivativeGains());
      setIntegralGains(gains.getIntegralGains(), gains.getMaximumIntegralError());
      setTangentialDampingGains(gains.getTangentialDampingGains());
      setMaxFeedbackAndFeedbackRate(gains.getMaximumFeedback(), gains.getMaximumFeedbackRate());
      setMaxDerivativeError(gains.getMaximumDerivativeError());
      setMaxProportionalError(gains.getMaximumProportionalError());
   }

   @Override
   public void set(SE3PIDGainsInterface gains)
   {
      set(gains.getPositionGains());
      set(gains.getOrientationGains());
   }

   public void createDerivativeGainUpdater(boolean updateNow)
   {
      VariableChangedListener kdUpdater = new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            derivativeGain.set(GainCalculator.computeDerivativeGain(proportionalGain.getDoubleValue(), dampingRatio.getDoubleValue()));
         }
      };

      proportionalGain.addVariableChangedListener(kdUpdater);
      dampingRatio.addVariableChangedListener(kdUpdater);

      if (updateNow)
         kdUpdater.variableChanged(null);
   }

   @Override
   public Matrix3DReadOnly createProportionalGainMatrix()
   {
      Matrix3D proportionalGainMatrix = new Matrix3D();

      for (int i = 0; i < 3; i++)
      {
         proportionalGain.addVariableChangedListener(new MatrixUpdater(i, i, proportionalGainMatrix));
      }

      proportionalGain.notifyVariableChangedListeners();
      return proportionalGainMatrix;
   }

   @Override
   public Matrix3DReadOnly createDerivativeGainMatrix()
   {
      Matrix3D derivativeGainMatrix = new Matrix3D();

      for (int i = 0; i < 3; i++)
      {
         derivativeGain.addVariableChangedListener(new MatrixUpdater(i, i, derivativeGainMatrix));
      }

      derivativeGain.notifyVariableChangedListeners();
      return derivativeGainMatrix;
   }

   @Override
   public Matrix3DReadOnly createIntegralGainMatrix()
   {
      Matrix3D integralGainMatrix = new Matrix3D();

      for (int i = 0; i < 3; i++)
      {
         integralGain.addVariableChangedListener(new MatrixUpdater(i, i, integralGainMatrix));
      }

      integralGain.notifyVariableChangedListeners();
      return integralGainMatrix;
   }

   @Override
   public void setProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      proportionalGain.set(proportionalGainX);
   }

   @Override
   public void setDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      derivativeGain.set(derivativeGainX);
   }

   @Override
   public void setIntegralGains(double integralGainX, double integralGainY, double integralGainZ, double maxIntegralError)
   {
      integralGain.set(integralGainX);
      this.maxIntegralError.set(maxIntegralError);
   }

   @Override
   public void setProportionalGains(double[] proportionalGains)
   {
      proportionalGain.set(proportionalGains[0]);
   }

   @Override
   public void setDerivativeGains(double[] derivativeGains)
   {
      derivativeGain.set(derivativeGains[0]);
   }

   @Override
   public void setIntegralGains(double[] integralGains, double maxIntegralError)
   {
      integralGain.set(integralGains[0]);
      this.maxIntegralError.set(maxIntegralError);
   }

   @Override
   public void setMaxFeedbackAndFeedbackRate(double maxFeedback, double maxFeedbackRate)
   {
      maximumFeedback.set(maxFeedback);
      maximumFeedbackRate.set(maxFeedbackRate);
   }

   @Override
   public void setTangentialDampingGains(TangentialDampingGains tangentialDampingGains)
   {
      this.tangentialDampingGains.set(tangentialDampingGains);
   }

   @Override
   public void setTangentialDampingGains(double kdReductionRatio, double parallelDampingDeadband, double positionErrorForMinKd)
   {
      this.tangentialDampingGains.set(kdReductionRatio, parallelDampingDeadband, positionErrorForMinKd);
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
   public TangentialDampingGains getTangentialDampingGains()
   {
      return tangentialDampingGains;
   }

   @Override
   public YoTangentialDampingGains getYoTangentialDampingGains()
   {
      return tangentialDampingGains;
   }

   @Override
   public YoDouble getYoMaximumFeedback()
   {
      return maximumFeedback;
   }

   @Override
   public YoDouble getYoMaximumFeedbackRate()
   {
      return maximumFeedbackRate;
   }

   @Override
   public YoDouble getYoMaximumDerivativeError()
   {
      return maxDerivativeError;
   }

   @Override
   public YoDouble getYoMaximumProportionalError()
   {
      return maxProportionalError;
   }

   private double[] tempPropotionalGains = new double[3];

   @Override
   public double[] getProportionalGains()
   {
      for (int i = 0; i < 3; i++)
         tempPropotionalGains[i] = proportionalGain.getDoubleValue();
      return tempPropotionalGains;
   }

   private double[] tempDerivativeGains = new double[3];

   @Override
   public double[] getDerivativeGains()
   {
      for (int i = 0; i < 3; i++)
         tempDerivativeGains[i] = derivativeGain.getDoubleValue();
      return tempDerivativeGains;
   }

   private double[] tempIntegralGains = new double[3];

   @Override
   public double[] getIntegralGains()
   {
      for (int i = 0; i < 3; i++)
         tempIntegralGains[i] = integralGain.getDoubleValue();
      return tempIntegralGains;
   }

   @Override
   public double getMaximumIntegralError()
   {
      return maxIntegralError.getDoubleValue();
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
