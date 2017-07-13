package us.ihmc.robotDataLogger.util;

import us.ihmc.util.PeriodicThreadScheduler;

public interface PeriodicThreadSchedulerFactory
{
   PeriodicThreadScheduler createPeriodicThreadScheduler(String name);
}
