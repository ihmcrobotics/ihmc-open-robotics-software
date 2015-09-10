package us.ihmc.commonWalkingControlModules.controlModules.foot;

import javax.vecmath.Matrix3d;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.yoUtilities.controllers.GainCalculator;
import us.ihmc.yoUtilities.controllers.MatrixUpdater;
import us.ihmc.yoUtilities.controllers.YoOrientationPIDGains;



public class YoFootOrientationGains implements YoOrientationPIDGains
{
   private final DoubleYoVariable proportionalXYGain, proportionalZGain;
   private final DoubleYoVariable derivativeXYGain, derivativeZGain;
   private final DoubleYoVariable dampingRatioXY, dampingRatioZ;

   private final DoubleYoVariable maximumAcceleration;
   private final DoubleYoVariable maximumJerk;

   public YoFootOrientationGains(String suffix, YoVariableRegistry registry)
   {
      proportionalXYGain = new DoubleYoVariable("kpXYAngular" + suffix, registry);
      proportionalZGain = new DoubleYoVariable("kpZAngular" + suffix, registry);
      derivativeXYGain = new DoubleYoVariable("kdXYAngular" + suffix, registry);
      derivativeZGain = new DoubleYoVariable("kdZAngular" + suffix, registry);
      dampingRatioXY = new DoubleYoVariable("zetaXYAngular" + suffix, registry);
      dampingRatioZ = new DoubleYoVariable("zetaZAngular" + suffix, registry);

      maximumAcceleration = new DoubleYoVariable("maximumAngularAcceleration" + suffix, registry);
      maximumJerk = new DoubleYoVariable("maximumAngularJerk" + suffix, registry);

      maximumAcceleration.set(Double.POSITIVE_INFINITY);
      maximumJerk.set(Double.POSITIVE_INFINITY);
   }

   public void reset()
   {
      proportionalXYGain.set(0.0);
      proportionalZGain.set(0.0);
      derivativeXYGain.set(0.0);
      derivativeZGain.set(0.0);
      dampingRatioXY.set(0.0);
      dampingRatioZ.set(0.0);
      maximumAcceleration.set(Double.POSITIVE_INFINITY);
      maximumJerk.set(Double.POSITIVE_INFINITY);
   }

   public Matrix3d createProportionalGainMatrix()
   {
      Matrix3d proportionalGainMatrix = new Matrix3d();

      proportionalXYGain.addVariableChangedListener(new MatrixUpdater(0, 0, proportionalGainMatrix));
      proportionalXYGain.addVariableChangedListener(new MatrixUpdater(1, 1, proportionalGainMatrix));
      proportionalZGain.addVariableChangedListener(new MatrixUpdater(2, 2, proportionalGainMatrix));

      proportionalXYGain.notifyVariableChangedListeners();
      proportionalZGain.notifyVariableChangedListeners();

      return proportionalGainMatrix;
   }

   public Matrix3d createDerivativeGainMatrix()
   {
      Matrix3d derivativeGainMatrix = new Matrix3d();

      derivativeXYGain.addVariableChangedListener(new MatrixUpdater(0, 0, derivativeGainMatrix));
      derivativeXYGain.addVariableChangedListener(new MatrixUpdater(1, 1, derivativeGainMatrix));
      derivativeZGain.addVariableChangedListener(new MatrixUpdater(2, 2, derivativeGainMatrix));

      derivativeXYGain.notifyVariableChangedListeners();
      derivativeZGain.notifyVariableChangedListeners();

      return derivativeGainMatrix;
   }

   public Matrix3d createIntegralGainMatrix()
   {
      return new Matrix3d();
   }

   public void createDerivativeGainUpdater(boolean updateNow)
   {
      VariableChangedListener kdXYUpdater = new VariableChangedListener()
      {
         public void variableChanged(YoVariable<?> v)
         {
            derivativeXYGain.set(GainCalculator.computeDerivativeGain(proportionalXYGain.getDoubleValue(), dampingRatioXY.getDoubleValue()));
         }
      };

      proportionalXYGain.addVariableChangedListener(kdXYUpdater);
      dampingRatioXY.addVariableChangedListener(kdXYUpdater);
      
      if (updateNow) kdXYUpdater.variableChanged(null);

      VariableChangedListener kdZUpdater = new VariableChangedListener()
      {
         public void variableChanged(YoVariable<?> v)
         {
            derivativeZGain.set(GainCalculator.computeDerivativeGain(proportionalZGain.getDoubleValue(), dampingRatioZ.getDoubleValue()));
         }
      };

      proportionalZGain.addVariableChangedListener(kdZUpdater);
      dampingRatioZ.addVariableChangedListener(kdZUpdater);
      
      if (updateNow) kdZUpdater.variableChanged(null);
   }

   public void setProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      proportionalXYGain.set(proportionalGainX);
      proportionalZGain.set(proportionalGainZ);
   }

   public void setProportionalGains(double proportionalGainXY, double proportionalGainZ)
   {
      proportionalXYGain.set(proportionalGainXY);
      proportionalZGain.set(proportionalGainZ);
   }

   public void setDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      derivativeXYGain.set(derivativeGainX);
      derivativeZGain.set(derivativeGainZ);
   }

   public void setDerivativeGains(double derivativeGainXY, double derivativeGainZ)
   {
      derivativeXYGain.set(derivativeGainXY);
      derivativeZGain.set(derivativeGainZ);
   }

   public void setDampingRatio(double dampingRatio)
   {
      this.dampingRatioXY.set(dampingRatio);
      this.dampingRatioZ.set(dampingRatio);
   }
   
   public void setDampingRatios(double dampingRatioXY, double dampingRatioZ)
   {
      this.dampingRatioXY.set(dampingRatioXY);
      this.dampingRatioZ.set(dampingRatioZ);
   }

   public void setIntegralGains(double integralGainX, double integralGainY, double integralGainZ, double maxIntegralError)
   {
   }

   public void setProportionalGains(double[] proportionalGains)
   {
      setProportionalGains(proportionalGains[0], proportionalGains[1], proportionalGains[2]);
   }

   public void setDerivativeGains(double[] derivativeGains)
   {
      setDerivativeGains(derivativeGains[0], derivativeGains[1], derivativeGains[2]);
   }

   public void setIntegralGains(double[] integralGains, double maxIntegralError)
   {
   }

   public void setMaximumAcceleration(double maxAcceleration)
   {
      this.maximumAcceleration.set(maxAcceleration);
   }

   public void setMaximumJerk(double maxJerk)
   {
      this.maximumJerk.set(maxJerk);
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
      tempPropotionalGains[0] = proportionalXYGain.getDoubleValue();
      tempPropotionalGains[1] = proportionalXYGain.getDoubleValue();
      tempPropotionalGains[2] = proportionalZGain.getDoubleValue();
      
      return tempPropotionalGains;
   }

   private double[] tempDerivativeGains = new double[3];
   public double[] getDerivativeGains()
   {
      tempDerivativeGains[0] = derivativeXYGain.getDoubleValue();
      tempDerivativeGains[1] = derivativeXYGain.getDoubleValue();
      tempDerivativeGains[2] = derivativeZGain.getDoubleValue();
      
      return tempDerivativeGains;
   }

   private double[] tempIntegralGains = new double[3];
   public double[] getIntegralGains()
   {
      return tempIntegralGains;
   }

   public double getMaximumIntegralError()
   {
      return 0.0;
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
