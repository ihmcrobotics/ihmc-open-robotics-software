package us.ihmc.robotics.controllers.pidGains;

import us.ihmc.robotics.controllers.GainCalculator;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class DampingUpdater implements VariableChangedListener
{
   private final YoDouble kp;
   private final YoDouble kd;
   private final YoDouble zeta;

   private final YoBoolean update;

   public DampingUpdater(YoDouble kp, YoDouble kd, YoDouble zeta, YoBoolean update)
   {
      this.kp = kp;
      this.kd = kd;
      this.zeta = zeta;
      this.update = update;
   }

   @Override
   public void variableChanged(YoVariable<?> v)
   {
      if (update.getBooleanValue())
      {
         // needs to update listeners in case a gain matrix is updated from this
         kd.set(GainCalculator.computeDerivativeGain(kp.getDoubleValue(), zeta.getDoubleValue()), true);
      }
   }
}
