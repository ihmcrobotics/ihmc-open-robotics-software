package us.ihmc.tools.thread;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
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
      ResettableExceptionHandlingExecutorService executor = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName());
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

   @Test
   public void testOneQueued()
   {
      StringBuilder output = new StringBuilder();

      ResettableExceptionHandlingExecutorService executor = MissingThreadTools.newSingleThreadExecutor("Test", false, 1);

      TypedNotification<String> resultOne = new TypedNotification<>();
      TypedNotification<String> resultTwo = new TypedNotification<>();
      Runnable runnableOne = () ->
      {
         output.append("a");
         ThreadTools.sleep(10);
         resultOne.set(output.toString());
      };
      Runnable runnableTwo = () ->
      {
         output.append("b");
         ThreadTools.sleep(10);
         resultTwo.set(output.toString());
      };

      executor.clearTaskQueue();
      executor.submit(runnableOne);
      executor.clearTaskQueue();
      executor.submit(runnableTwo);

      resultOne.blockingPoll();
      assertEquals("a", resultOne.read());
      resultTwo.blockingPoll();
      assertEquals("ab", resultTwo.read());
   }

   @Test
   public void testSkipItemInQueue()
   {
      StringBuilder output = new StringBuilder();

      ResettableExceptionHandlingExecutorService executor = MissingThreadTools.newSingleThreadExecutor("Test", false, 1);

      TypedNotification<String> resultOne = new TypedNotification<>();
      TypedNotification<String> resultTwo = new TypedNotification<>();
      TypedNotification<String> resultThree = new TypedNotification<>();
      Runnable runnableOne = () ->
      {
         output.append("a");
         ThreadTools.sleep(10);
         resultOne.set(output.toString());
      };
      Runnable runnableTwo = () ->
      {
         output.append("b");
         ThreadTools.sleep(10);
         resultTwo.set(output.toString());
      };
      Runnable runnableThree = () ->
      {
         output.append("c");
         ThreadTools.sleep(10);
         resultThree.set(output.toString());
      };

      executor.clearTaskQueue();
      executor.submit(runnableOne);
      executor.clearTaskQueue();
      executor.submit(runnableTwo); // this one should get skipped
      executor.clearTaskQueue();
      executor.submit(runnableThree);

      resultOne.blockingPoll();
      assertEquals("a", resultOne.read());
      resultThree.blockingPoll();
      assertEquals("ac", resultThree.read());
   }
}
