package us.ihmc.avatar.logging;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.Conversions;
import us.ihmc.robotDataLogger.dataBuffers.RegistrySendBufferBuilder;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.LockSupport;

public class IntraprocessYoVariableLoggerV2Test
{
   @Test
   public void testLogging()
   {
      YoRegistry registry = new YoRegistry("testRegistry");

      for (int i = 0; i < 30000; i++)
      {
         new YoDouble("double_" + i, registry);
      }

      final double dt = 0.01;

      List<RegistrySendBufferBuilder> bufferBuilders = new ArrayList<>();
      bufferBuilders.add(new RegistrySendBufferBuilder(registry));

      IntraprocessYoVariableLoggerV2 logger = new IntraprocessYoVariableLoggerV2(bufferBuilders, dt, getClass().getSimpleName());

      logger.create();

      long startTime = System.nanoTime();
      for (int i = 0; i < 100; i++)
      {
         logger.update(System.nanoTime() - startTime);

         LockSupport.parkNanos(Conversions.secondsToNanoseconds(dt));
      }

      logger.destroy();
   }
}
