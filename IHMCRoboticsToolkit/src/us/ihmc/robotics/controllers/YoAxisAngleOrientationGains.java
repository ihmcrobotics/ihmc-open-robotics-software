package us.ihmc.robotics.controllers;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoAxisAngleOrientationGains implements YoOrientationPIDGainsInterface
{
   private static final String[] directionNames = new String[] {"x", "y", "z"};

   private final YoDouble[] proportionalGains = new YoDouble[3];
   private final YoDouble[] derivativeGains = new YoDouble[3];
   private final YoDouble[] integralGains = new YoDouble[3];
   private final YoDouble[] dampingRatios;

   private final YoDouble maxIntegralError;
   private final YoDouble maxDerivativeError;
   private final YoDouble maxProportionalError;

   private final YoDouble maxFeedback;
   private final YoDouble maxFeedbackRate;

   public YoAxisAngleOrientationGains(String prefix, YoVariableRegistry registry)
   {
      this(prefix, false, registry);
   }

   public YoAxisAngleOrientationGains(String prefix, boolean createDampingRatio, YoVariableRegistry registry)
   {
      String baseProportionalGainName = prefix + "OrientationProportionalGain";
      String baseDerivativeGainName = prefix + "OrientationDerivativeGain";
      String baseIntegralGainName = prefix + "OrientationIntegralGain";

      for (int i = 0; i < 3; i++)
      {
         proportionalGains[i] = new YoDouble(baseProportionalGainName + directionNames[i], registry);
         derivativeGains[i] = new YoDouble(baseDerivativeGainName + directionNames[i], registry);
         integralGains[i] = new YoDouble(baseIntegralGainName + directionNames[i], registry);
      }

      if (createDampingRatio)
      {
         String baseDampingRatioName = prefix + "OrientationZeta";

         dampingRatios = new YoDouble[3];
         for (int i = 0; i < 3; i++)
         {
            dampingRatios[i] = new YoDouble(baseDampingRatioName + directionNames[i], registry);
         }
      }
      else
      {
         dampingRatios = null;
      }

      maxIntegralError = new YoDouble(prefix + "OrientationMaxIntegralError", registry);
      maxDerivativeError = new YoDouble(prefix + "OrientationMaxDerivativeError", registry);
      maxProportionalError = new YoDouble(prefix + "OrientationMaxProportionalError", registry);

      maxFeedback = new YoDouble(prefix + "OrientationMaxFeedback", registry);
      maxFeedbackRate = new YoDouble(prefix + "OrientationMaxFeedbackRate", registry);

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

      if (dampingRatios != null)
      {
         for (int i = 0; i < 3; i++)
            dampingRatios[i].set(0.0);
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

   public void setDampingRatio(double dampingRatio)
   {
      setDampingRatios(dampingRatio, dampingRatio, dampingRatio);
   }

   public void setDampingRatios(double dampingRatioX, double dampingRatioY, double dampingRatioZ)
   {
      dampingRatios[0].set(dampingRatioX);
      dampingRatios[1].set(dampingRatioY);
      dampingRatios[2].set(dampingRatioZ);
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
   public YoDouble getYoMaximumFeedback()
   {
      return maxFeedback;
   }

   @Override
   public YoDouble getYoMaximumFeedbackRate()
   {
      return maxFeedbackRate;
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

   public void createDerivativeGainUpdater(boolean updateNow)
   {
      for (int i = 0; i < 3; i++)
      {
         int index = i;

         VariableChangedListener kdUpdater = new VariableChangedListener()
         {
            @Override
            public void variableChanged(YoVariable<?> v)
            {
               derivativeGains[index].set(GainCalculator.computeDerivativeGain(proportionalGains[index].getDoubleValue(),
                                                                               dampingRatios[index].getDoubleValue()));
            }
         };

         proportionalGains[i].addVariableChangedListener(kdUpdater);
         dampingRatios[i].addVariableChangedListener(kdUpdater);

         if (updateNow)
            kdUpdater.variableChanged(null);
      }
   }
}
