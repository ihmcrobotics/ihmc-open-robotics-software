package us.ihmc.tools.thread;

import us.ihmc.commons.thread.ThreadTools;

import java.util.concurrent.*;

/**
 * TODO: Add shutdown exception limit?
 */
public class ResettableSingleThreadExecutor
{
   private final String prefix;
   private ExecutorService executor;

   public ResettableSingleThreadExecutor(String prefix)
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
