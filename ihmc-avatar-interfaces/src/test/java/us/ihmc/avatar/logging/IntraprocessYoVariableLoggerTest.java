package us.ihmc.avatar.logging;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.Conversions;
import us.ihmc.robotDataLogger.dataBuffers.RegistrySendBufferBuilder;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoLong;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.LockSupport;

public class IntraprocessYoVariableLoggerTest
{
   public static final int TICKS = 10000;

   @Test
   public void testLoggingSpeed() throws IOException
   {
      YoRegistry mainRegistry = new YoRegistry("testRegistry");

      // Create a whole bunch of fake YoVariables to be used in the logger
      for (int i = 0; i < 5000; i++)
      {
         YoLong longVariable = new YoLong("long_" + i, mainRegistry);
         longVariable.set(i);
      }

      double dt = 0.001;

      List<RegistrySendBufferBuilder> bufferBuilders = new ArrayList<>();
      bufferBuilders.add(new RegistrySendBufferBuilder(mainRegistry));

      IntraprocessYoVariableLogger intraprocessYoVariableLogger = new IntraprocessYoVariableLogger(bufferBuilders, dt, getClass().getSimpleName());
      intraprocessYoVariableLogger.create();

      long logDurationTotal = 0;

      for (int i = 0; i < TICKS; i++)
      {
         long tickStart = System.nanoTime();
         intraprocessYoVariableLogger.update(i);
         long logDuration = System.nanoTime() - tickStart;
         logDurationTotal += logDuration;
         long tickSleep = Conversions.secondsToNanoseconds(dt) - logDuration;
         LockSupport.parkNanos(tickSleep);
      }

      System.out.println("Average per update: " + TimeUnit.NANOSECONDS.toMicros((logDurationTotal / TICKS)) + " us");
   }
}
