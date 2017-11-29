package us.ihmc.robotics.controllers;

import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class PIDController extends AbstractPIDController
{
   final PDController pdController;
   final YoDouble integralGain;
   final YoDouble maxIntegralError;
   final YoDouble maxOutput;
   final YoDouble integralLeakRatio;
   
   public PIDController(String suffix, YoVariableRegistry registry)
   {
      super(suffix, registry);
      
      pdController = new PDController(suffix, registry);

      integralGain = new YoDouble("ki_" + suffix, registry);
      integralGain.set(0.0);

      maxIntegralError = new YoDouble("maxIntegralError_" + suffix, registry);
      maxIntegralError.set(Double.POSITIVE_INFINITY);
      
      maxOutput = new YoDouble("maxOutput_" + suffix, registry);
      maxOutput.set(Double.POSITIVE_INFINITY);

      
      integralLeakRatio = new YoDouble("leak_" + suffix, registry);
      integralLeakRatio.set(1.0);

      addLeakRatioClipper();
   }

   private void addLeakRatioClipper()
   {
      VariableChangedListener leakRatioClipper = new VariableChangedListener()
      {
         @Override public void notifyOfVariableChange(YoVariable<?> v)
         {
            integralLeakRatio.set(MathTools.clamp(integralLeakRatio.getDoubleValue(), 0.0, 1.0), false);
         }
      };

      integralLeakRatio.addVariableChangedListener(leakRatioClipper);
   }

   public PIDController(YoDouble proportionalGain, YoDouble integralGain, YoDouble derivativeGain, YoDouble maxIntegralError,
         String suffix, YoVariableRegistry registry)
   {
      super(suffix, registry);
      
      pdController = new PDController(proportionalGain, derivativeGain, suffix, registry);
      this.integralGain = integralGain;
      this.maxIntegralError = maxIntegralError;
      
      maxOutput = new YoDouble("maxOutput_" + suffix, registry);
      maxOutput.set(Double.POSITIVE_INFINITY);

      
      integralLeakRatio = new YoDouble("leak_" + suffix, registry);
      integralLeakRatio.set(1.0);
      
      addLeakRatioClipper();
   }

   public PIDController(YoPIDGains yoPIDGains, String suffix, YoVariableRegistry registry)
   {
      super(suffix, registry);
      
      pdController = new PDController(yoPIDGains, suffix, registry);
      this.integralGain = yoPIDGains.getYoKi();
      this.maxIntegralError = yoPIDGains.getYoMaxIntegralError();
      this.maxOutput = yoPIDGains.getYoMaximumOutput();

      integralLeakRatio = yoPIDGains.getYoIntegralLeakRatio();
   }
   
   public void setMaximumOutputLimit(double max)
   {
      if( max <=0 )
         maxOutput.set(Double.POSITIVE_INFINITY);
      else
         maxOutput.set( max );
   }

   public void setProportionalGain(double proportionalGain)
   {
      pdController.setProportionalGain(proportionalGain);
   }

   public void setDerivativeGain(double derivativeGain)
   {
      pdController.setDerivativeGain(derivativeGain);
   }

   public void setPositionDeadband(double deadband)
   {
      pdController.setPositionDeadband(deadband);
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

   @Override
   protected AbstractPDController getPDController()
   {
      return pdController;
   }

   @Override
   public double getMaximumOutputLimit()
   {
      return maxOutput.getDoubleValue();
   }

   @Override
   public double getIntegralGain()
   {
      return integralGain.getDoubleValue();
   }

   @Override
   public double getMaxIntegralError()
   {
      return maxIntegralError.getDoubleValue();
   }

   @Override
   public double getIntegralLeakRatio()
   {
      return integralLeakRatio.getDoubleValue();
   }
}
