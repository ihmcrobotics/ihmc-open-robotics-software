package us.ihmc.robotics.controllers.pidGains;

import us.ihmc.robotics.controllers.GainCalculator;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class ZetaUpdater implements VariableChangedListener
{
   private final YoDouble kp;
   private final YoDouble kd;
   private final YoDouble zeta;

   private final YoBoolean update;

   public ZetaUpdater(YoDouble kp, YoDouble kd, YoDouble zeta, YoBoolean update)
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
         // will not update listeners to avoid a loop when updating kd
         zeta.set(GainCalculator.computeDampingRatio(kp.getDoubleValue(), kd.getDoubleValue()), false);
      }
   }
}
