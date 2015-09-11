package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

/**
 *
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class DeltaLimitedYoVariable extends DoubleYoVariable
{
   private final DoubleYoVariable maxDelta;
   private final DoubleYoVariable actual;
   private final DoubleYoVariable desired;
   private final BooleanYoVariable isLimitingActive;

   public DeltaLimitedYoVariable(String name, YoVariableRegistry registry, double maxDelta)
   {
      super(name, registry);

      this.maxDelta = new DoubleYoVariable(name + "MaxAllowedDelta", registry);
      this.maxDelta.set(Math.abs(maxDelta));
      this.actual = new DoubleYoVariable(name + "Actual", registry);
      this.desired = new DoubleYoVariable(name + "Desired", registry);
      this.isLimitingActive = new BooleanYoVariable(name + "IsLimitingActive", registry);
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
