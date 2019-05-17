package us.ihmc.robotics.time;

import us.ihmc.commons.Conversions;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public class ThreadTimer
{
   private final YoLong tick;
   private final YoDouble dt;
   private final YoDouble timer;

   private long lastStartTime;

   public ThreadTimer(String name, YoVariableRegistry registry)
   {
      tick = new YoLong(name + "Tick", registry);
      dt = new YoDouble(name + "DT", registry);
      timer = new YoDouble(name + "Timer", registry);
   }

   public void start()
   {
      long startTime = System.nanoTime();
      if (lastStartTime != 0)
         dt.set(Conversions.nanosecondsToMilliseconds((double) (startTime - lastStartTime)));
      lastStartTime = startTime;
      tick.increment();
   }

   public void stop()
   {
      timer.set(Conversions.nanosecondsToMilliseconds((double) (System.nanoTime() - lastStartTime)));
   }
}
