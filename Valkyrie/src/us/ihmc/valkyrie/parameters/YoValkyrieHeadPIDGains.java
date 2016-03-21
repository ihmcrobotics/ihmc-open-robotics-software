package us.ihmc.valkyrie.parameters;

import javax.vecmath.Matrix3d;

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

   private final DoubleYoVariable maximumAcceleration;
   private final DoubleYoVariable maximumJerk;

   public YoValkyrieHeadPIDGains(String suffix, YoVariableRegistry registry)
   {
      proportionalXGain = new DoubleYoVariable("kpXAngular" + suffix, registry);
      proportionalYZGain = new DoubleYoVariable("kpYZAngular" + suffix, registry);
      derivativeXGain = new DoubleYoVariable("kdXAngular" + suffix, registry);
      derivativeYZGain = new DoubleYoVariable("kdYZAngular" + suffix, registry);
      dampingRatioX = new DoubleYoVariable("zetaXAngular" + suffix, registry);
      dampingRatioYZ = new DoubleYoVariable("zetaYZAngular" + suffix, registry);

      maximumAcceleration = new DoubleYoVariable("maximumAngularAcceleration" + suffix, registry);
      maximumJerk = new DoubleYoVariable("maximumAngularJerk" + suffix, registry);

      maximumAcceleration.set(Double.POSITIVE_INFINITY);
      maximumJerk.set(Double.POSITIVE_INFINITY);
   }

   public void reset()
   {
      proportionalXGain.set(0.0);
      proportionalYZGain.set(0.0);
      derivativeXGain.set(0.0);
      derivativeYZGain.set(0.0);
      dampingRatioX.set(0.0);
      dampingRatioYZ.set(0.0);
      maximumAcceleration.set(Double.POSITIVE_INFINITY);
      maximumJerk.set(Double.POSITIVE_INFINITY);
   }

   @Override
   public Matrix3d createProportionalGainMatrix()
   {
      Matrix3d proportionalGainMatrix = new Matrix3d();

      proportionalXGain.addVariableChangedListener(new MatrixUpdater(0, 0, proportionalGainMatrix));
      proportionalYZGain.addVariableChangedListener(new MatrixUpdater(1, 1, proportionalGainMatrix));
      proportionalYZGain.addVariableChangedListener(new MatrixUpdater(2, 2, proportionalGainMatrix));

      proportionalXGain.notifyVariableChangedListeners();
      proportionalYZGain.notifyVariableChangedListeners();

      return proportionalGainMatrix;
   }

   @Override
   public Matrix3d createDerivativeGainMatrix()
   {
      Matrix3d derivativeGainMatrix = new Matrix3d();

      derivativeXGain.addVariableChangedListener(new MatrixUpdater(0, 0, derivativeGainMatrix));
      derivativeYZGain.addVariableChangedListener(new MatrixUpdater(1, 1, derivativeGainMatrix));
      derivativeYZGain.addVariableChangedListener(new MatrixUpdater(2, 2, derivativeGainMatrix));

      derivativeXGain.notifyVariableChangedListeners();
      derivativeYZGain.notifyVariableChangedListeners();

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

   public void setMaximumAcceleration(double maxAcceleration)
   {
      maximumAcceleration.set(maxAcceleration);
   }

   public void setMaximumJerk(double maxJerk)
   {
      maximumJerk.set(maxJerk);
   }

   @Override
   public void setMaxAccelerationAndJerk(double maxAcceleration, double maxJerk)
   {
      maximumAcceleration.set(maxAcceleration);
      maximumJerk.set(maxJerk);
   }

   @Override
   public void set(OrientationPIDGainsInterface gains)
   {
      setProportionalGains(gains.getProportionalGains());
      setDerivativeGains(gains.getDerivativeGains());
      setIntegralGains(gains.getIntegralGains(), gains.getMaximumIntegralError());
      setMaxAccelerationAndJerk(gains.getMaximumAcceleration(), gains.getMaximumJerk());
   }

   @Override
   public DoubleYoVariable getYoMaximumAcceleration()
   {
      return maximumAcceleration;
   }

   @Override
   public DoubleYoVariable getYoMaximumJerk()
   {
      return maximumJerk;
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
   public double getMaximumAcceleration()
   {
      return maximumAcceleration.getDoubleValue();
   }

   @Override
   public double getMaximumJerk()
   {
      return maximumJerk.getDoubleValue();
   }
}
