package us.ihmc.robotics.controllers;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.AngleTools;



public class PDController
{
   private final DoubleYoVariable proportionalGain;
   private final DoubleYoVariable derivativeGain;
   private final DoubleYoVariable positionError;
   private final DoubleYoVariable rateError;

   private final DoubleYoVariable actionP;
   private final DoubleYoVariable actionD;

   public PDController(String suffix, YoVariableRegistry registry)
   {
      proportionalGain = new DoubleYoVariable("kp_" + suffix,  registry);
      proportionalGain.set(0.0);

      derivativeGain = new DoubleYoVariable("kd_" + suffix,  registry);
      derivativeGain.set(0.0);

      positionError = new DoubleYoVariable("positionError_" + suffix,  registry);
      positionError.set(0.0);

      rateError = new DoubleYoVariable("rateError_" + suffix,  registry);
      rateError.set(0.0);
      
      actionP = new DoubleYoVariable("action_P_" + suffix, registry);
      actionP.set(0.0);
      
      actionD = new DoubleYoVariable("action_D_" + suffix, registry);
      actionD.set(0.0);
   }

   public PDController(DoubleYoVariable proportionalGain, DoubleYoVariable derivativeGain, String suffix, YoVariableRegistry registry)
   {
      this.proportionalGain = proportionalGain;
      this.derivativeGain = derivativeGain;

      positionError = new DoubleYoVariable("positionError_" + suffix,  registry);
      positionError.set(0.0);

      rateError = new DoubleYoVariable("rateError_" + suffix,  registry);
      rateError.set(0.0);
      
      actionP = new DoubleYoVariable("action_P_" + suffix, registry);
      actionP.set(0.0);
      
      actionD = new DoubleYoVariable("action_D_" + suffix, registry);
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

   public void setProportionalGain(double proportionalGain)
   {
      this.proportionalGain.set(proportionalGain);
   }

   public void setDerivativeGain(double derivativeGain)
   {
      this.derivativeGain.set(derivativeGain);
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
      positionError.set(desiredPosition - currentPosition);
      rateError.set(desiredRate - currentRate);
      
      actionP.set(proportionalGain.getDoubleValue() * positionError.getDoubleValue());
      actionD.set(derivativeGain.getDoubleValue() * rateError.getDoubleValue());

      return actionP.getDoubleValue() + actionD.getDoubleValue();
   }

   public double computeForAngles(double currentPosition, double desiredPosition, double currentRate, double desiredRate)
   {
//      System.out.println("PGain: " + proportionalGain.getDoubleValue() + "DGain: " + derivativeGain.getDoubleValue());
      positionError.set(AngleTools.computeAngleDifferenceMinusPiToPi(desiredPosition, currentPosition));
      rateError.set(desiredRate - currentRate);
      
      actionP.set(proportionalGain.getDoubleValue() * positionError.getDoubleValue());
      actionD.set(derivativeGain.getDoubleValue() * rateError.getDoubleValue());

      return actionP.getDoubleValue() + actionD.getDoubleValue();
   }
}
