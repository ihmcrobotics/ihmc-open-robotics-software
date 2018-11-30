package us.ihmc.robotics.controllers.stiction;

import us.ihmc.robotics.math.filters.RateLimitedYoVariable;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class StictionCompensator
{
   private final YoDouble desiredTorqueStictionLimitFactor;

   private final YoDouble stictionCompensationLimit;
   private final YoDouble stictionCompensationRate;
   private final RateLimitedYoVariable stictionCompensation;

   private final StictionModel stictionModel;
   private double desiredTorque;

   public StictionCompensator(String prefix, StictionModel stictionModel, double controlDT, YoVariableRegistry parentRegistry)
   {
      this.stictionModel = stictionModel;

      YoVariableRegistry registry = new YoVariableRegistry(prefix + getClass().getSimpleName());

      desiredTorqueStictionLimitFactor = new YoDouble(prefix + "_DesiredTorqueStictionLimitFactor", registry);

      stictionCompensationLimit = new YoDouble(prefix + "_StictionCompensationLimit", registry);
      stictionCompensationRate = new YoDouble(prefix + "_StictionCompensationRate", registry);
      stictionCompensation = new RateLimitedYoVariable(prefix + "_StictionCompensation", registry, stictionCompensationRate, controlDT);
      stictionCompensation.update(0.0);

      desiredTorqueStictionLimitFactor.set(2.0);
      stictionCompensationRate.set(20.0);

      parentRegistry.addChild(registry);
   }

   public void setDesiredTorque(double desiredTorque)
   {
      this.desiredTorque = desiredTorque;
   }

   public void resetStictionCompensation()
   {
      this.stictionCompensationLimit.set(0.0);
      this.stictionCompensation.set(0.0);
   }

   public double computeStictionCompensation()
   {
      double sign = Math.signum(desiredTorque);

      double stictionMagnitude = stictionModel.getStictionMagnitude();
      stictionCompensationLimit.set(Math.min(sign * desiredTorque * desiredTorqueStictionLimitFactor.getDoubleValue(), stictionMagnitude));
      stictionCompensation.update(sign * stictionCompensationLimit.getDoubleValue());

      return stictionCompensation.getDoubleValue();
   }

   public double getStictionCompensation()
   {
      return stictionCompensation.getDoubleValue();
   }

   // for testing
   public double getDesiredTorque()
   {
      return desiredTorque;
   }
}
