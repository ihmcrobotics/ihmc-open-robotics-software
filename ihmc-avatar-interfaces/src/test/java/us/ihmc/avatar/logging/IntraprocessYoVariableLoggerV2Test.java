package us.ihmc.avatar.logging;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.Conversions;
import us.ihmc.robotDataLogger.dataBuffers.RegistrySendBufferBuilder;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.LockSupport;

public class IntraprocessYoVariableLoggerV2Test
{
   @Test
   public void testLogging()
   {
      YoRegistry registry = new YoRegistry("testRegistry");
      YoInteger testInt = new YoInteger("testInt", registry);

      final double dt = 0.01;

      List<RegistrySendBufferBuilder> bufferBuilders = new ArrayList<>();
      bufferBuilders.add(new RegistrySendBufferBuilder(registry));

      IntraprocessYoVariableLoggerV2 logger = new IntraprocessYoVariableLoggerV2(bufferBuilders, dt, getClass().getSimpleName());

      logger.create();

      long startTime = System.nanoTime();
      for (int i = 0; i < 100; i++)
      {
         testInt.set(i);

         logger.update(System.nanoTime() - startTime);

         LockSupport.parkNanos(Conversions.secondsToNanoseconds(dt));
      }

      logger.destroy();
   }
}
