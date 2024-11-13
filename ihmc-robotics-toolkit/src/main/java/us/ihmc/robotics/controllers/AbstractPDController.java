package us.ihmc.robotics.controllers;

import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.math.DeadbandTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class AbstractPDController
{
   protected final YoDouble positionError;
   protected final YoDouble rateError;
   protected final YoDouble actionP;
   protected final YoDouble actionD;

   protected final DoubleProvider proportionalGain;
   protected final DoubleProvider derivativeGain;
   protected final DoubleProvider positionDeadband;

   protected AbstractPDController(DoubleProvider proportionalGain, DoubleProvider derivativeGain, DoubleProvider positionDeadband, String suffix, YoRegistry registry)
   {
      this.proportionalGain = proportionalGain;
      this.derivativeGain = derivativeGain;
      this.positionDeadband = positionDeadband;

      positionError = new YoDouble("positionError_" + suffix, registry);
      positionError.set(0.0);

      rateError = new YoDouble("rateError_" + suffix, registry);
      rateError.set(0.0);

      actionP = new YoDouble("action_P_" + suffix, registry);
      actionP.set(0.0);

      actionD = new YoDouble("action_D_" + suffix, registry);
      actionD.set(0.0);
   }

   public double getProportionalGain()
   {
      return proportionalGain.getValue();
   }

   public double getDerivativeGain()
   {
      return derivativeGain.getValue();
   }

   public double getPositionDeadband()
   {
      return positionDeadband.getValue();
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
      positionError.set(DeadbandTools.applyDeadband(positionDeadband.getValue(), desiredPosition - currentPosition));
      rateError.set(desiredRate - currentRate);

      actionP.set(proportionalGain.getValue() * positionError.getDoubleValue());
      actionD.set(derivativeGain.getValue() * rateError.getDoubleValue());

      return actionP.getDoubleValue() + actionD.getDoubleValue();
   }

   public double computeForAngles(double currentPosition, double desiredPosition, double currentRate, double desiredRate)
   {
      //      System.out.println("PGain: " + proportionalGain.getDoubleValue() + "DGain: " + derivativeGain.getDoubleValue());
      this.positionError.set(DeadbandTools.applyDeadband(positionDeadband.getValue(), AngleTools.computeAngleDifferenceMinusPiToPi(desiredPosition, currentPosition)));
      rateError.set(desiredRate - currentRate);

      actionP.set(proportionalGain.getValue() * positionError.getDoubleValue());
      actionD.set(derivativeGain.getValue() * rateError.getDoubleValue());

      return actionP.getDoubleValue() + actionD.getDoubleValue();
   }
}
