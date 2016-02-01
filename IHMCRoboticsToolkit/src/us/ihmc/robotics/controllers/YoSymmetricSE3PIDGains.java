package us.ihmc.robotics.controllers;

import javax.vecmath.Matrix3d;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;


public class YoSymmetricSE3PIDGains implements YoSE3PIDGains, YoPositionPIDGains, YoOrientationPIDGains
{
   private final DoubleYoVariable proportionalGain;
   private final DoubleYoVariable derivativeGain;
   private final DoubleYoVariable dampingRatio;
   private final DoubleYoVariable integralGain;
   private final DoubleYoVariable maxIntegralError;

   private final DoubleYoVariable maximumAcceleration;
   private final DoubleYoVariable maximumJerk;

   public YoSymmetricSE3PIDGains(String suffix, YoVariableRegistry registry)
   {
      proportionalGain = new DoubleYoVariable("kp" + suffix, registry);
      derivativeGain = new DoubleYoVariable("kd" + suffix, registry);
      dampingRatio = new DoubleYoVariable("zeta" + suffix, registry);
      integralGain = new DoubleYoVariable("ki" + suffix, registry);

      maxIntegralError = new DoubleYoVariable("maxIntegralError" + suffix, registry);
      maximumAcceleration = new DoubleYoVariable("maximumAcceleration" + suffix, registry);
      maximumJerk = new DoubleYoVariable("maximumJerk" + suffix, registry);

      maximumAcceleration.set(Double.POSITIVE_INFINITY);
      maximumJerk.set(Double.POSITIVE_INFINITY);
   }

   public void reset()
   {
      proportionalGain.set(0.0);
      derivativeGain.set(0.0);
      dampingRatio.set(0.0);
      integralGain.set(0.0);
      
      maxIntegralError.set(0.0);
      maximumAcceleration.set(Double.POSITIVE_INFINITY);
      maximumJerk.set(Double.POSITIVE_INFINITY);
   }

   public YoPositionPIDGains getPositionGains()
   {
      return this;
   }

   public YoOrientationPIDGains getOrientationGains()
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

   public void setMaximumAcceleration(double maxAcceleration)
   {
      this.maximumAcceleration.set(maxAcceleration);
   }

   public void setMaximumJerk(double maxJerk)
   {
      this.maximumJerk.set(maxJerk);
   }

   public void createDerivativeGainUpdater(boolean updateNow)
   {
      VariableChangedListener kdUpdater = new VariableChangedListener()
      {
         public void variableChanged(YoVariable<?> v)
         {
            derivativeGain.set(GainCalculator.computeDerivativeGain(proportionalGain.getDoubleValue(), dampingRatio.getDoubleValue()));
         }
      };

      proportionalGain.addVariableChangedListener(kdUpdater);
      dampingRatio.addVariableChangedListener(kdUpdater);
      
      if (updateNow) kdUpdater.variableChanged(null);
   }

   public Matrix3d createProportionalGainMatrix()
   {
      Matrix3d proportionalGainMatrix = new Matrix3d();

      for (int i = 0; i < 3; i++)
      {
         proportionalGain.addVariableChangedListener(new MatrixUpdater(i, i, proportionalGainMatrix));
      }

      proportionalGain.notifyVariableChangedListeners();
      return proportionalGainMatrix;
   }

   public Matrix3d createDerivativeGainMatrix()
   {
      Matrix3d derivativeGainMatrix = new Matrix3d();

      for (int i = 0; i < 3; i++)
      {
         derivativeGain.addVariableChangedListener(new MatrixUpdater(i, i, derivativeGainMatrix));
      }

      derivativeGain.notifyVariableChangedListeners();
      return derivativeGainMatrix;
   }

   public Matrix3d createIntegralGainMatrix()
   {
      Matrix3d integralGainMatrix = new Matrix3d();

      for (int i = 0; i < 3; i++)
      {
         integralGain.addVariableChangedListener(new MatrixUpdater(i, i, integralGainMatrix));
      }

      integralGain.notifyVariableChangedListeners();
      return integralGainMatrix;
   }

   public void setProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      proportionalGain.set(proportionalGainX);
   }

   public void setDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      derivativeGain.set(derivativeGainX);
   }

   public void setIntegralGains(double integralGainX, double integralGainY, double integralGainZ, double maxIntegralError)
   {
      integralGain.set(integralGainX);
      this.maxIntegralError.set(maxIntegralError);
   }

   public void setProportionalGains(double[] proportionalGains)
   {
      proportionalGain.set(proportionalGains[0]);
   }

   public void setDerivativeGains(double[] derivativeGains)
   {
      derivativeGain.set(derivativeGains[0]);
   }

   public void setIntegralGains(double[] integralGains, double maxIntegralError)
   {
      integralGain.set(integralGains[0]);
      this.maxIntegralError.set(maxIntegralError);
   }

   public void setMaxAccelerationAndJerk(double maxAcceleration, double maxJerk)
   {
      this.maximumAcceleration.set(maxAcceleration);
      this.maximumJerk.set(maxJerk);
   }

   public DoubleYoVariable getYoMaximumAcceleration()
   {
      return maximumAcceleration;
   }

   public DoubleYoVariable getYoMaximumJerk()
   {
      return maximumJerk;
   }

   private double[] tempPropotionalGains = new double[3];
   public double[] getProportionalGains()
   {
      for (int i = 0; i < 3; i++)
         tempPropotionalGains[i] = proportionalGain.getDoubleValue();
      return tempPropotionalGains;
   }

   private double[] tempDerivativeGains = new double[3];
   public double[] getDerivativeGains()
   {
      for (int i = 0; i < 3; i++)
         tempDerivativeGains[i] = derivativeGain.getDoubleValue();
      return tempDerivativeGains;
   }

   private double[] tempIntegralGains = new double[3];
   public double[] getIntegralGains()
   {
      for (int i = 0; i < 3; i++)
         tempIntegralGains[i] = integralGain.getDoubleValue();
      return tempIntegralGains;
   }

   public double getMaximumIntegralError()
   {
      return maxIntegralError.getDoubleValue();
   }

   public double getMaximumAcceleration()
   {
      return maximumAcceleration.getDoubleValue();
   }

   public double getMaximumJerk()
   {
      return maximumJerk.getDoubleValue();
   }
}
