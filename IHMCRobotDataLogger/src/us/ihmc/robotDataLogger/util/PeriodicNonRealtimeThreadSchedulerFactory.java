package us.ihmc.robotDataLogger.util;

import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;
import us.ihmc.util.PeriodicThreadScheduler;

public class PeriodicNonRealtimeThreadSchedulerFactory implements PeriodicThreadSchedulerFactory
{

   @Override
   public PeriodicThreadScheduler createPeriodicThreadScheduler(String name)
   {
      return new PeriodicNonRealtimeThreadScheduler(name);
   }

}
