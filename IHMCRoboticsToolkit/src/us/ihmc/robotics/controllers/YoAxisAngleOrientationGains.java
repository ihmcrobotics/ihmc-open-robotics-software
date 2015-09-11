package us.ihmc.robotics.controllers;

import javax.vecmath.Matrix3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;


public class YoAxisAngleOrientationGains implements YoOrientationPIDGains
{
   private static final String[] directionNames = new String[] { "x", "y", "z" };

   private final DoubleYoVariable[] proportionalGains = new DoubleYoVariable[3];
   private final DoubleYoVariable[] derivativeGains = new DoubleYoVariable[3];
   private final DoubleYoVariable[] integralGains = new DoubleYoVariable[3];
   private final DoubleYoVariable maxIntegralError;

   private final DoubleYoVariable maxAcceleration;
   private final DoubleYoVariable maxJerk;

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

      maxAcceleration = new DoubleYoVariable(prefix + "OrientationMaxAcceleration", registry);
      maxJerk = new DoubleYoVariable(prefix + "OrientationMaxJerk", registry);

      maxAcceleration.set(Double.POSITIVE_INFINITY);
      maxJerk.set(Double.POSITIVE_INFINITY);
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
      maxAcceleration.set(Double.POSITIVE_INFINITY);
      maxJerk.set(Double.POSITIVE_INFINITY);
   }

   public Matrix3d createProportionalGainMatrix()
   {
      Matrix3d proportionalGainMatrix = new Matrix3d();

      for (int i = 0; i < 3; i++)
      {
         proportionalGains[i].addVariableChangedListener(new MatrixUpdater(i, i, proportionalGainMatrix));
         proportionalGains[i].notifyVariableChangedListeners();
      }

      return proportionalGainMatrix;
   }

   public Matrix3d createDerivativeGainMatrix()
   {
      Matrix3d derivativeGainMatrix = new Matrix3d();

      for (int i = 0; i < 3; i++)
      {
         derivativeGains[i].addVariableChangedListener(new MatrixUpdater(i, i, derivativeGainMatrix));
         derivativeGains[i].notifyVariableChangedListeners();
      }

      return derivativeGainMatrix;
   }

   public Matrix3d createIntegralGainMatrix()
   {
      Matrix3d integralGainMatrix = new Matrix3d();

      for (int i = 0; i < 3; i++)
      {
         integralGains[i].addVariableChangedListener(new MatrixUpdater(i, i, integralGainMatrix));
         integralGains[i].notifyVariableChangedListeners();
      }

      return integralGainMatrix;
   }

   public void setProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      proportionalGains[0].set(proportionalGainX);
      proportionalGains[1].set(proportionalGainY);
      proportionalGains[2].set(proportionalGainZ);
   }

   public void setDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      derivativeGains[0].set(derivativeGainX);
      derivativeGains[1].set(derivativeGainY);
      derivativeGains[2].set(derivativeGainZ);
   }

   public void setIntegralGains(double integralGainX, double integralGainY, double integralGainZ, double maxIntegralError)
   {
      integralGains[0].set(integralGainX);
      integralGains[1].set(integralGainY);
      integralGains[2].set(integralGainZ);
      
      this.maxIntegralError.set(maxIntegralError);
   }

   public void setProportionalGains(double[] proportionalGains)
   {
      for (int i = 0; i < proportionalGains.length; i++)
      {
         this.proportionalGains[i].set(proportionalGains[i]);
      }
   }

   public void setDerivativeGains(double[] derivativeGains)
   {
      for (int i = 0; i < derivativeGains.length; i++)
      {
         this.derivativeGains[i].set(derivativeGains[i]);
      }
   }

   public void setIntegralGains(double[] integralGains, double maxIntegralError)
   {
      for (int i = 0; i < integralGains.length; i++)
      {
         this.integralGains[i].set(integralGains[i]);
      }
      
      this.maxIntegralError.set(maxIntegralError);
   }

   public void setMaxAccelerationAndJerk(double maxAcceleration, double maxJerk)
   {
      this.maxAcceleration.set(maxAcceleration);
      this.maxJerk.set(maxJerk);
   }

   public DoubleYoVariable getYoMaximumAcceleration()
   {
      return maxAcceleration;
   }

   public DoubleYoVariable getYoMaximumJerk()
   {
      return maxJerk;
   }

   private double[] tempPropotionalGains = new double[3];
   public double[] getProportionalGains()
   {
      for (int i = 0; i < 3; i++)
         tempPropotionalGains[i] = proportionalGains[i].getDoubleValue();
      return tempPropotionalGains;
   }

   private double[] tempDerivativeGains = new double[3];
   public double[] getDerivativeGains()
   {
      for (int i = 0; i < 3; i++)
         tempDerivativeGains[i] = derivativeGains[i].getDoubleValue();
      return tempDerivativeGains;
   }

   private double[] tempIntegralGains = new double[3];
   public double[] getIntegralGains()
   {
      for (int i = 0; i < 3; i++)
         tempIntegralGains[i] = integralGains[i].getDoubleValue();
      return tempIntegralGains;
   }

   public double getMaximumIntegralError()
   {
      return maxIntegralError.getDoubleValue();
   }

   public double getMaximumAcceleration()
   {
      return maxAcceleration.getDoubleValue();
   }

   public double getMaximumJerk()
   {
      return maxJerk.getDoubleValue();
   }
}
