package us.ihmc.avatar.logging;

import org.junit.jupiter.api.Test;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoLong;

public class IntraprocessYoVariableLoggerTest
{
   public static final int iterations = 100;

   @Test
   public void testLoggingSpeed()
   {
      YoRegistry mainRegistry = new YoRegistry("testRegistry");

      // Create a whole bunch of fake YoVariables to be used in the logger
      for (int i = 0; i < 5000; i++)
      {
         YoLong longYoVaraible = new YoLong("longYoVaraible" + i, mainRegistry);
         longYoVaraible.set(System.currentTimeMillis());
      }

      IntraprocessYoVariableLogger intraprocessYoVariableLogger = new IntraprocessYoVariableLogger("ss", mainRegistry, 100000, 0.01);
      intraprocessYoVariableLogger.start();

      long startTime = System.nanoTime();

      for (int i = 0; i < iterations; i++)
      {
         intraprocessYoVariableLogger.update(System.currentTimeMillis());
      }

      long endTime = System.nanoTime();
      double totalMicros = (endTime - startTime) / 1_000.0;
      double avgMicros = totalMicros / iterations;

      System.out.println("Total time: " + totalMicros + " µs");
      System.out.println("Average per update: " + avgMicros + " µs");
   }
}
