package us.ihmc.robotics.controllers.pidGains;

import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class DampingUpdater implements YoVariableChangedListener
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
      update.addListener(this);
   }

   @Override
   public void changed(YoVariable v)
   {
      if (update.getBooleanValue())
      {
         kd.set(GainCalculator.computeDerivativeGain(kp.getDoubleValue(), zeta.getDoubleValue()));
      }
   }
}
