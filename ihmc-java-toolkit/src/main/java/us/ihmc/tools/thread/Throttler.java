package us.ihmc.tools.thread;

import us.ihmc.tools.Timer;

public class Throttler
{
   private final Timer timer = new Timer();

   public Throttler()
   {
      timer.reset();
   }

   public boolean run(double period)
   {
      boolean run = !timer.isRunning(period);
      if (run)
      {
         timer.reset();
      }
      return run;
   }
}
