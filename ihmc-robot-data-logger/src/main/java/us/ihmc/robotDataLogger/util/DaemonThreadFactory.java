package us.ihmc.robotDataLogger.util;

import java.util.concurrent.ThreadFactory;

import us.ihmc.commons.thread.ThreadTools;

public class DaemonThreadFactory
{
   private DaemonThreadFactory()
   {
   }

   public static ThreadFactory getNamedDaemonThreadFactory(String name)
   {
      ThreadFactory threadFactory = ThreadTools.getNamedThreadFactory(name);

      return runnable ->
      {
         Thread newThread = threadFactory.newThread(runnable);
         newThread.setDaemon(true);
         return newThread;
      };
   }
}
