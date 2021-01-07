package us.ihmc.tools.thread;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;

import java.util.concurrent.atomic.AtomicBoolean;

import static org.junit.jupiter.api.Assertions.*;

public class MissingThreadToolsTest
{
   @Test
   public void testSingleScheduleThreadWithThrownException()
   {
      LogTools.info("Begin test");
      int[] ints = new int[1];
      SaferExecutorService executor = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName());
      AtomicBoolean assertionRan = new AtomicBoolean();
      executor.submit(() ->
                      {
                         LogTools.info("ints[0] = {}", ints[0]);
                         LogTools.info("ints[1] = {}", ints[1]);
                      }, (exception, cancelled, interrupted) ->
                      {
                         if (!cancelled && !interrupted)
                         {
                            DefaultExceptionHandler.MESSAGE_AND_STACKTRACE.handleException(exception);
                            assertTrue(exception instanceof ArrayIndexOutOfBoundsException);
                            assertionRan.set(true);
                         }
                      });

      ThreadTools.sleepSeconds(0.2);

      assertTrue(assertionRan.get());
   }
}
