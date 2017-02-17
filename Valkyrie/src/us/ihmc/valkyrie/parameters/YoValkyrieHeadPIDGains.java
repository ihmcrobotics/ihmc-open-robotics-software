package us.ihmc.valkyrie.parameters;

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

public class YoValkyrieHeadPIDGains implements YoOrientationPIDGainsInterface
{
   private final DoubleYoVariable proportionalXGain, proportionalYZGain;
   private final DoubleYoVariable derivativeXGain, derivativeYZGain;
   private final DoubleYoVariable dampingRatioX, dampingRatioYZ;

   private final DoubleYoVariable maxDerivativeError;
   private final DoubleYoVariable maxProportionalError;

   private final DoubleYoVariable maximumFeedback;
   private final DoubleYoVariable maximumFeedbackRate;

   public YoValkyrieHeadPIDGains(String suffix, YoVariableRegistry registry)
   {
      proportionalXGain = new DoubleYoVariable("kpXAngular" + suffix, registry);
      proportionalYZGain = new DoubleYoVariable("kpYZAngular" + suffix, registry);
      derivativeXGain = new DoubleYoVariable("kdXAngular" + suffix, registry);
      derivativeYZGain = new DoubleYoVariable("kdYZAngular" + suffix, registry);
      dampingRatioX = new DoubleYoVariable("zetaXAngular" + suffix, registry);
      dampingRatioYZ = new DoubleYoVariable("zetaYZAngular" + suffix, registry);

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
      proportionalYZGain.set(0.0);
      derivativeXGain.set(0.0);
      derivativeYZGain.set(0.0);
      dampingRatioX.set(0.0);
      dampingRatioYZ.set(0.0);
      maximumFeedback.set(Double.POSITIVE_INFINITY);
      maximumFeedbackRate.set(Double.POSITIVE_INFINITY);
      maxDerivativeError.set(Double.POSITIVE_INFINITY);
      maxProportionalError.set(Double.POSITIVE_INFINITY);
   }

   @Override
   public Matrix3DReadOnly createProportionalGainMatrix()
   {
      Matrix3D proportionalGainMatrix = new Matrix3D();

      proportionalXGain.addVariableChangedListener(new MatrixUpdater(0, 0, proportionalGainMatrix));
      proportionalYZGain.addVariableChangedListener(new MatrixUpdater(1, 1, proportionalGainMatrix));
      proportionalYZGain.addVariableChangedListener(new MatrixUpdater(2, 2, proportionalGainMatrix));

      proportionalXGain.notifyVariableChangedListeners();
      proportionalYZGain.notifyVariableChangedListeners();

      return proportionalGainMatrix;
   }

   @Override
   public Matrix3DReadOnly createDerivativeGainMatrix()
   {
      Matrix3D derivativeGainMatrix = new Matrix3D();

      derivativeXGain.addVariableChangedListener(new MatrixUpdater(0, 0, derivativeGainMatrix));
      derivativeYZGain.addVariableChangedListener(new MatrixUpdater(1, 1, derivativeGainMatrix));
      derivativeYZGain.addVariableChangedListener(new MatrixUpdater(2, 2, derivativeGainMatrix));

      derivativeXGain.notifyVariableChangedListeners();
      derivativeYZGain.notifyVariableChangedListeners();

      return derivativeGainMatrix;
   }

   @Override
   public Matrix3DReadOnly createIntegralGainMatrix()
   {
      return new Matrix3D();
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

      VariableChangedListener kdYZUpdater = new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            derivativeYZGain.set(GainCalculator.computeDerivativeGain(proportionalYZGain.getDoubleValue(), dampingRatioYZ.getDoubleValue()));
         }
      };

      proportionalYZGain.addVariableChangedListener(kdYZUpdater);
      dampingRatioYZ.addVariableChangedListener(kdYZUpdater);

      if (updateNow)
         kdYZUpdater.variableChanged(null);
   }

   @Override
   public void setProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      proportionalXGain.set(proportionalGainX);
      proportionalYZGain.set(proportionalGainY);
   }

   public void setProportionalGains(double proportionalGainX, double proportionalGainYZ)
   {
      proportionalXGain.set(proportionalGainX);
      proportionalYZGain.set(proportionalGainYZ);
   }

   @Override
   public void setDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      derivativeXGain.set(derivativeGainX);
      derivativeYZGain.set(derivativeGainY);
   }

   public void setDerivativeGains(double derivativeGainX, double derivativeGainYZ)
   {
      derivativeXGain.set(derivativeGainX);
      derivativeYZGain.set(derivativeGainYZ);
   }

   public void setDampingRatio(double dampingRatio)
   {
      dampingRatioX.set(dampingRatio);
      dampingRatioYZ.set(dampingRatio);
   }

   public void setDampingRatios(double dampingRatioX, double dampingRatioYZ)
   {
      this.dampingRatioX.set(dampingRatioX);
      this.dampingRatioYZ.set(dampingRatioYZ);
   }

   @Override
   public void setIntegralGains(double integralGainX, double integralGainY, double integralGainZ, double maxIntegralError)
   {
   }

   @Override
   public void setProportionalGains(double[] proportionalGains)
   {
      setProportionalGains(proportionalGains[0], proportionalGains[1], proportionalGains[2]);
   }

   @Override
   public void setDerivativeGains(double[] derivativeGains)
   {
      setDerivativeGains(derivativeGains[0], derivativeGains[1], derivativeGains[2]);
   }

   @Override
   public void setIntegralGains(double[] integralGains, double maxIntegralError)
   {
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

   private double[] tempPropotionalGains = new double[3];

   @Override
   public double[] getProportionalGains()
   {
      tempPropotionalGains[0] = proportionalXGain.getDoubleValue();
      tempPropotionalGains[1] = proportionalYZGain.getDoubleValue();
      tempPropotionalGains[2] = proportionalYZGain.getDoubleValue();

      return tempPropotionalGains;
   }

   private double[] tempDerivativeGains = new double[3];

   @Override
   public double[] getDerivativeGains()
   {
      tempDerivativeGains[0] = derivativeXGain.getDoubleValue();
      tempDerivativeGains[1] = derivativeYZGain.getDoubleValue();
      tempDerivativeGains[2] = derivativeYZGain.getDoubleValue();

      return tempDerivativeGains;
   }


   private double[] tempIntegralGains = new double[3];

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
