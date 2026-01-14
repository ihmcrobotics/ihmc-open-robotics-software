package us.ihmc.avatar.factory;

import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.Task;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class ChangingRateControlTask<C> extends Task<C>
{
   public ChangingRateControlTask(YoDouble dtProvider, double schedulerDt)
   {
      super((int) Math.round(dtProvider.getValue() / schedulerDt));

      dtProvider.addListener(v -> setDivisor((int) Math.round(dtProvider.getValue() / schedulerDt)));
   }
}
