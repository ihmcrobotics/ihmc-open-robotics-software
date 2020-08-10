package us.ihmc.humanoidBehaviors.lookAndStep;

import us.ihmc.commons.thread.ThreadTools;

import java.util.concurrent.*;

/**
 * TODO: Add shutdown exception limit?
 */
public class SingleThreadExecutor
{
   private final String prefix;
   private ExecutorService executor;

   public SingleThreadExecutor(String prefix)
   {
      this.prefix = prefix;

      recreate();
   }

   private void recreate()
   {
      executor = ThreadTools.newSingleThreadExecutor(prefix);
   }

   public void queueExecution(Runnable runnable)
   {
      executor.execute(runnable);
   }

   public void interruptAndReset()
   {
      executor.shutdownNow();
      recreate();
   }
}
