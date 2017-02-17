package us.ihmc.robotics.controllers;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class YoAxisAngleOrientationGains implements YoOrientationPIDGainsInterface
{
   private static final String[] directionNames = new String[] {"x", "y", "z"};

   private final DoubleYoVariable[] proportionalGains = new DoubleYoVariable[3];
   private final DoubleYoVariable[] derivativeGains = new DoubleYoVariable[3];
   private final DoubleYoVariable[] integralGains = new DoubleYoVariable[3];

   private final DoubleYoVariable maxIntegralError;
   private final DoubleYoVariable maxDerivativeError;
   private final DoubleYoVariable maxProportionalError;

   private final DoubleYoVariable maxFeedback;
   private final DoubleYoVariable maxFeedbackRate;

   public YoAxisAngleOrientationGains(String prefix, YoVariableRegistry registry)
   {
      String baseProportionalGainName = prefix + "OrientationProportionalGain";
      String baseDerivativeGainName = prefix + "OrientationDerivativeGain";
      String baseIntegralGainName = prefix + "OrientationIntegralGain";

      for (int i = 0; i < 3; i++)
      {
         proportionalGains[i] = new DoubleYoVariable(baseProportionalGainName + directionNames[i], registry);
         derivativeGains[i] = new DoubleYoVariable(baseDerivativeGainName + directionNames[i], registry);
         integralGains[i] = new DoubleYoVariable(baseIntegralGainName + directionNames[i], registry);
      }

      maxIntegralError = new DoubleYoVariable(prefix + "OrientationMaxIntegralError", registry);
      maxDerivativeError = new DoubleYoVariable(prefix + "OrientationMaxDerivativeError", registry);
      maxProportionalError = new DoubleYoVariable(prefix + "OrientationMaxProportionalError", registry);

      maxFeedback = new DoubleYoVariable(prefix + "OrientationMaxFeedback", registry);
      maxFeedbackRate = new DoubleYoVariable(prefix + "OrientationMaxFeedbackRate", registry);

      maxFeedback.set(Double.POSITIVE_INFINITY);
      maxFeedbackRate.set(Double.POSITIVE_INFINITY);
      maxDerivativeError.set(Double.POSITIVE_INFINITY);
      maxProportionalError.set(Double.POSITIVE_INFINITY);
   }

   public void reset()
   {
      for (int i = 0; i < proportionalGains.length; i++)
      {
         proportionalGains[i].set(0.0);
         derivativeGains[i].set(0.0);
         integralGains[i].set(0.0);
      }

      maxIntegralError.set(0.0);
      maxDerivativeError.set(Double.POSITIVE_INFINITY);
      maxProportionalError.set(Double.POSITIVE_INFINITY);

      maxFeedback.set(Double.POSITIVE_INFINITY);
      maxFeedbackRate.set(Double.POSITIVE_INFINITY);
   }

   @Override
   public Matrix3DReadOnly createProportionalGainMatrix()
   {
      Matrix3D proportionalGainMatrix = new Matrix3D();

      for (int i = 0; i < 3; i++)
      {
         proportionalGains[i].addVariableChangedListener(new MatrixUpdater(i, i, proportionalGainMatrix));
         proportionalGains[i].notifyVariableChangedListeners();
      }

      return proportionalGainMatrix;
   }

   @Override
   public Matrix3DReadOnly createDerivativeGainMatrix()
   {
      Matrix3D derivativeGainMatrix = new Matrix3D();

      for (int i = 0; i < 3; i++)
      {
         derivativeGains[i].addVariableChangedListener(new MatrixUpdater(i, i, derivativeGainMatrix));
         derivativeGains[i].notifyVariableChangedListeners();
      }

      return derivativeGainMatrix;
   }

   @Override
   public Matrix3DReadOnly createIntegralGainMatrix()
   {
      Matrix3D integralGainMatrix = new Matrix3D();

      for (int i = 0; i < 3; i++)
      {
         integralGains[i].addVariableChangedListener(new MatrixUpdater(i, i, integralGainMatrix));
         integralGains[i].notifyVariableChangedListeners();
      }

      return integralGainMatrix;
   }

   @Override
   public void setProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      proportionalGains[0].set(proportionalGainX);
      proportionalGains[1].set(proportionalGainY);
      proportionalGains[2].set(proportionalGainZ);
   }

   @Override
   public void setDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      derivativeGains[0].set(derivativeGainX);
      derivativeGains[1].set(derivativeGainY);
      derivativeGains[2].set(derivativeGainZ);
   }

   @Override
   public void setIntegralGains(double integralGainX, double integralGainY, double integralGainZ, double maxIntegralError)
   {
      integralGains[0].set(integralGainX);
      integralGains[1].set(integralGainY);
      integralGains[2].set(integralGainZ);

      this.maxIntegralError.set(maxIntegralError);
   }

   @Override
   public void setProportionalGains(double[] proportionalGains)
   {
      for (int i = 0; i < proportionalGains.length; i++)
      {
         this.proportionalGains[i].set(proportionalGains[i]);
      }
   }

   @Override
   public void setDerivativeGains(double[] derivativeGains)
   {
      for (int i = 0; i < derivativeGains.length; i++)
      {
         this.derivativeGains[i].set(derivativeGains[i]);
      }
   }

   @Override
   public void setIntegralGains(double[] integralGains, double maxIntegralError)
   {
      for (int i = 0; i < integralGains.length; i++)
      {
         this.integralGains[i].set(integralGains[i]);
      }

      this.maxIntegralError.set(maxIntegralError);
   }

   @Override
   public void setMaxFeedbackAndFeedbackRate(double maxFeedback, double maxFeedbackRate)
   {
      this.maxFeedback.set(maxFeedback);
      this.maxFeedbackRate.set(maxFeedbackRate);
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
      return maxFeedback;
   }

   @Override
   public DoubleYoVariable getYoMaximumFeedbackRate()
   {
      return maxFeedbackRate;
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
      for (int i = 0; i < 3; i++)
         tempPropotionalGains[i] = proportionalGains[i].getDoubleValue();
      return tempPropotionalGains;
   }

   private double[] tempDerivativeGains = new double[3];

   @Override
   public double[] getDerivativeGains()
   {
      for (int i = 0; i < 3; i++)
         tempDerivativeGains[i] = derivativeGains[i].getDoubleValue();
      return tempDerivativeGains;
   }

   private double[] tempIntegralGains = new double[3];

   @Override
   public double[] getIntegralGains()
   {
      for (int i = 0; i < 3; i++)
         tempIntegralGains[i] = integralGains[i].getDoubleValue();
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
      return maxFeedback.getDoubleValue();
   }

   @Override
   public double getMaximumFeedbackRate()
   {
      return maxFeedbackRate.getDoubleValue();
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
