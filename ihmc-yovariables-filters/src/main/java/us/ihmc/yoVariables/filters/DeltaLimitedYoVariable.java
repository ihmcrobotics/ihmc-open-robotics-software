package us.ihmc.yoVariables.filters;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This variable stores data in such a way that its value is guaranteed to be within a certain delta of the desired value.
 */
public class DeltaLimitedYoVariable extends YoDouble
{
   private final YoDouble maxDelta;
   private final YoDouble actual;
   private final YoDouble desired;
   private final YoBoolean isLimitingActive;

   public DeltaLimitedYoVariable(String name, YoRegistry registry, double maxDelta)
   {
      super(name, registry);

      this.maxDelta = new YoDouble(name + "MaxAllowedDelta", registry);
      this.maxDelta.set(Math.abs(maxDelta));
      this.actual = new YoDouble(name + "Actual", registry);
      this.desired = new YoDouble(name + "Desired", registry);
      this.isLimitingActive = new YoBoolean(name + "IsLimitingActive", registry);
      isLimitingActive.set(false);
   }

   public void setMaxDelta(double maxDelta)
   {
      this.maxDelta.set(Math.abs(maxDelta));
   }

   public void updateOutput(double actual, double desired)
   {
      this.desired.set(desired);
      this.actual.set(actual);
      updateOutput();
   }

   public boolean isLimitingActive()
   {
      return isLimitingActive.getBooleanValue();
   }

   private void updateOutput()
   {
      double actualDoubleValue = actual.getDoubleValue();
      double desiredDoubleValue = desired.getDoubleValue();
      double maxDeltaDoubleValue = Math.abs(maxDelta.getDoubleValue());
      double rawDelta = actualDoubleValue - desiredDoubleValue;
      double sign = Math.signum(rawDelta);
      double requestedDelta = Math.abs(rawDelta);
      double overshoot = maxDeltaDoubleValue - requestedDelta;

      if(overshoot < 0)
      {
         desiredDoubleValue = actualDoubleValue - maxDeltaDoubleValue * sign;
         isLimitingActive.set(true);
      }
      else
      {
         isLimitingActive.set(false);
      }

      this.set(desiredDoubleValue);
   }
}
