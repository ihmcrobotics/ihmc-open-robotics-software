package us.ihmc.robotics.controllers;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.geometry.AngleTools;

public class PDController
{
   private final YoDouble proportionalGain;
   private final YoDouble derivativeGain;
   private final YoDouble positionError;
   private final YoDouble rateError;
   private final YoDouble positionDeadband;

   private final YoDouble actionP;
   private final YoDouble actionD;

   public PDController(String suffix, YoVariableRegistry registry)
   {
      proportionalGain = new YoDouble("kp_" + suffix,  registry);
      proportionalGain.set(0.0);

      derivativeGain = new YoDouble("kd_" + suffix,  registry);
      derivativeGain.set(0.0);

      positionDeadband = new YoDouble("positionDeadband_" + suffix, registry);
      positionDeadband.set(0.0);

      positionError = new YoDouble("positionError_" + suffix,  registry);
      positionError.set(0.0);

      rateError = new YoDouble("rateError_" + suffix,  registry);
      rateError.set(0.0);
      
      actionP = new YoDouble("action_P_" + suffix, registry);
      actionP.set(0.0);
      
      actionD = new YoDouble("action_D_" + suffix, registry);
      actionD.set(0.0);
   }

   public PDController(YoDouble proportionalGain, YoDouble derivativeGain, String suffix, YoVariableRegistry registry)
   {
      this.proportionalGain = proportionalGain;
      this.derivativeGain = derivativeGain;

      positionDeadband = new YoDouble("positionDeadband_" + suffix, registry);
      positionDeadband.set(0.0);

      positionError = new YoDouble("positionError_" + suffix,  registry);
      positionError.set(0.0);

      rateError = new YoDouble("rateError_" + suffix,  registry);
      rateError.set(0.0);
      
      actionP = new YoDouble("action_P_" + suffix, registry);
      actionP.set(0.0);
      
      actionD = new YoDouble("action_D_" + suffix, registry);
      actionD.set(0.0);
   }

   public PDController(YoDouble proportionalGain, YoDouble derivativeGain, YoDouble positionDeadband, String suffix, YoVariableRegistry registry)
   {
      this.proportionalGain = proportionalGain;
      this.derivativeGain = derivativeGain;
      this.positionDeadband = positionDeadband;

      positionError = new YoDouble("positionError_" + suffix,  registry);
      positionError.set(0.0);

      rateError = new YoDouble("rateError_" + suffix,  registry);
      rateError.set(0.0);

      actionP = new YoDouble("action_P_" + suffix, registry);
      actionP.set(0.0);

      actionD = new YoDouble("action_D_" + suffix, registry);
      actionD.set(0.0);
   }

   public PDController(YoPDGains pdGains, String suffix, YoVariableRegistry registry)
   {
      this.proportionalGain = pdGains.getYoKp();
      this.derivativeGain = pdGains.getYoKd();
      this.positionDeadband = pdGains.getPositionDeadband();

      positionError = new YoDouble("positionError_" + suffix,  registry);
      positionError.set(0.0);

      rateError = new YoDouble("rateError_" + suffix,  registry);
      rateError.set(0.0);

      actionP = new YoDouble("action_P_" + suffix, registry);
      actionP.set(0.0);

      actionD = new YoDouble("action_D_" + suffix, registry);
      actionD.set(0.0);
   }

   public double getProportionalGain()
   {
      return proportionalGain.getDoubleValue();
   }

   public double getDerivativeGain()
   {
      return derivativeGain.getDoubleValue();
   }

   public double getPositionDeadband()
   {
      return positionDeadband.getDoubleValue();
   }

   public void setProportionalGain(double proportionalGain)
   {
      this.proportionalGain.set(proportionalGain);
   }

   public void setDerivativeGain(double derivativeGain)
   {
      this.derivativeGain.set(derivativeGain);
   }

   public void setPositionDeadband(double positionDeadband)
   {
      this.positionDeadband.set(positionDeadband);
   }

   public double getPositionError()
   {
      return positionError.getDoubleValue();
   }

   public double getRateError()
   {
      return rateError.getDoubleValue();
   }

   public double compute(double currentPosition, double desiredPosition, double currentRate, double desiredRate)
   {
      positionError.set(applyDeadband(desiredPosition - currentPosition));
      rateError.set(desiredRate - currentRate);
      
      actionP.set(proportionalGain.getDoubleValue() * positionError.getDoubleValue());
      actionD.set(derivativeGain.getDoubleValue() * rateError.getDoubleValue());

      return actionP.getDoubleValue() + actionD.getDoubleValue();
   }

   public double computeForAngles(double currentPosition, double desiredPosition, double currentRate, double desiredRate)
   {
//      System.out.println("PGain: " + proportionalGain.getDoubleValue() + "DGain: " + derivativeGain.getDoubleValue());
      this.positionError.set(applyDeadband(AngleTools.computeAngleDifferenceMinusPiToPi(desiredPosition, currentPosition)));
      rateError.set(desiredRate - currentRate);
      
      actionP.set(proportionalGain.getDoubleValue() * positionError.getDoubleValue());
      actionD.set(derivativeGain.getDoubleValue() * rateError.getDoubleValue());

      return actionP.getDoubleValue() + actionD.getDoubleValue();
   }

   private double applyDeadband(double positionError)
   {
      if (positionError >= positionDeadband.getDoubleValue())
         positionError -= positionDeadband.getDoubleValue();
      else if (positionError <= -positionDeadband.getDoubleValue())
         positionError += positionDeadband.getDoubleValue();
      else
         positionError = 0.0;

      return positionError;
   }
}
