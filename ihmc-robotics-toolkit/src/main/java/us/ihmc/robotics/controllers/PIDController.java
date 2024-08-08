package us.ihmc.robotics.controllers;

import us.ihmc.commons.MathTools;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPIDGains;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class PIDController extends AbstractPIDController
{
   private final YoDouble proportionalGain;
   private final YoDouble integralGain;
   private final YoDouble derivativeGain;
   private final YoDouble positionDeadband;
   private final YoDouble maxIntegralError;
   private final YoDouble maxFeedback;
   private final YoDouble integralLeakRatio;

   public PIDController(String suffix, YoRegistry registry)
   {
      this(new YoDouble("kp_" + suffix, registry),
            new YoDouble("ki_" + suffix, registry),
            new YoDouble("kd_" + suffix, registry),
            new YoDouble("maxIntegralError_" + suffix, registry),
            suffix,
           registry);

      proportionalGain.set(0.0);
      integralGain.set(0.0);
      derivativeGain.set(0.0);

      maxIntegralError.set(Double.POSITIVE_INFINITY);
   }

   public PIDController(YoDouble proportionalGain, YoDouble integralGain, YoDouble derivativeGain, YoDouble maxIntegralError, String suffix,
                        YoRegistry registry)
   {
      this(proportionalGain,
            integralGain,
            derivativeGain,
            new YoDouble("positionDeadband_" + suffix, registry),
            maxIntegralError,
            new YoDouble("maxOutput_" + suffix, registry),
            new YoDouble("leak_" + suffix, registry),
            suffix,
            registry);

      positionDeadband.set(0.0);
      maxFeedback.set(Double.POSITIVE_INFINITY);
      integralLeakRatio.set(1.0);
   }


   public PIDController(YoPIDGains yoPIDGains, String suffix, YoRegistry registry)
   {
      this(yoPIDGains.getYoKp(),
            yoPIDGains.getYoKi(),
            yoPIDGains.getYoKd(),
            yoPIDGains.getYoPositionDeadband(),
            yoPIDGains.getYoMaxIntegralError(),
            yoPIDGains.getYoMaximumFeedback(),
            yoPIDGains.getYoIntegralLeakRatio(),
            suffix,
           registry);
   }

   public PIDController(YoDouble proportionalGain, YoDouble integralGain, YoDouble derivativeGain, YoDouble positionDeadband, YoDouble maxIntegralError, YoDouble maxFeedback,
                        YoDouble integralLeakRatio, String suffix,
                        YoRegistry registry)
   {
      super(proportionalGain, integralGain, derivativeGain, positionDeadband, maxIntegralError, maxFeedback, integralLeakRatio, suffix, registry);

      this.proportionalGain = (YoDouble) super.proportionalGain;
      this.integralGain = (YoDouble) super.integralGain;
      this.derivativeGain = (YoDouble) super.derivativeGain;
      this.positionDeadband = (YoDouble) super.positionDeadband;
      this.maxIntegralError = (YoDouble) super.maxIntegralError;
      this.maxFeedback = (YoDouble) super.maxFeedback;
      this.integralLeakRatio = (YoDouble) super.integralLeakRatio;

      addLeakRatioClipper();
   }


   private void addLeakRatioClipper()
   {
      YoVariableChangedListener leakRatioClipper = new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
         {
            integralLeakRatio.set(MathTools.clamp(integralLeakRatio.getDoubleValue(), 0.0, 1.0), false);
         }
      };

      integralLeakRatio.addListener(leakRatioClipper);
   }

   public void setMaximumOutputLimit(double max)
   {
      if (max <= 0.0)
         maxFeedback.set(Double.POSITIVE_INFINITY);
      else
         maxFeedback.set(max);
   }

   public void setProportionalGain(double proportionalGain)
   {
      this.proportionalGain.set(proportionalGain);
   }

   public void setDerivativeGain(double derivativeGain)
   {
      this.derivativeGain.set(derivativeGain);
   }

   public void setPositionDeadband(double deadband)
   {
      this.positionDeadband.set(deadband);
   }

   public void setIntegralGain(double integralGain)
   {
      this.integralGain.set(integralGain);
   }

   public void setIntegralLeakRatio(double integralLeakRatio)
   {
      this.integralLeakRatio.set(MathTools.clamp(integralLeakRatio, 0.0, 1.0));
   }

   public void setMaxIntegralError(double maxIntegralError)
   {
      this.maxIntegralError.set(maxIntegralError);
   }

   public void setGains(PIDGainsReadOnly gains)
   {
      setProportionalGain(gains.getKp());
      setDerivativeGain(gains.getKd());
      setPositionDeadband(gains.getPositionDeadband());
      setMaximumOutputLimit(gains.getMaximumFeedback());
      setIntegralLeakRatio(gains.getIntegralLeakRatio());
      setIntegralGain(gains.getKi());
      setMaxIntegralError(gains.getMaxIntegralError());
   }
}
