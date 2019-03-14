package us.ihmc.robotDataLogger.util;

import us.ihmc.util.PeriodicThreadScheduler;
import us.ihmc.util.PeriodicThreadSchedulerFactory;

public class PeriodicGCFreeNonRealtimeThreadSchedulerFactory implements PeriodicThreadSchedulerFactory
{
   @Override
   public PeriodicThreadScheduler createPeriodicThreadScheduler(String name)
   {
      return new PeriodicGCFreeNonRealtimeThreadScheduler(name);
   }
}
