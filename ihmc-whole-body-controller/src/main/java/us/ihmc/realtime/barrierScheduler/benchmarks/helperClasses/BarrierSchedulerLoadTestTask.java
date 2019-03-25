package us.ihmc.realtime.barrierScheduler.benchmarks.helperClasses;

import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.tasks.CopyableContextTask;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public abstract class BarrierSchedulerLoadTestTask extends CopyableContextTask
{
   public BarrierSchedulerLoadTestTask(long divisor)
   {
      super(divisor);
   }

   public abstract BarrierSchedulerLoadTestData getLoadTestData();

   public abstract void doTimingReporting();
}
