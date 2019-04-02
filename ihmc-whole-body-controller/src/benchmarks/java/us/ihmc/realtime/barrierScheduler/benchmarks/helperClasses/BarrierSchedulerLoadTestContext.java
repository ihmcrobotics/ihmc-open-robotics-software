package us.ihmc.realtime.barrierScheduler.benchmarks.helperClasses;

import org.apache.commons.math3.util.Pair;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.tasks.BindingContext;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class BarrierSchedulerLoadTestContext extends BindingContext
{
   public BarrierSchedulerLoadTestContext(Pair<BarrierSchedulerLoadTestTask, BarrierSchedulerLoadTestTask>... bindings)
   {
      if (bindings != null && bindings.length > 0)
      {
         for (Pair<BarrierSchedulerLoadTestTask, BarrierSchedulerLoadTestTask> binding : bindings)
         {
            bind(binding.getFirst().getLoadTestData(), binding.getSecond().getLoadTestData());
         }
      }
   }
}
