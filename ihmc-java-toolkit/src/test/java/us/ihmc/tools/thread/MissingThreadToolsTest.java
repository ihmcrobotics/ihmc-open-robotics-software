package us.ihmc.tools.thread;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.log.LogTools;

import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService.MESSAGE_AND_TRACE_WITH_THREAD_NAME;

public class MissingThreadToolsTest
{
   @Test
   public void testSingleScheduleThreadWithThrownException()
   {
      LogTools.info("Begin test");
      int[] ints = new int[1];
      int queueSize = -1;
      boolean daemon= false;
      ResettableExceptionHandlingExecutorService executor = MissingThreadTools.newSingleThreadExecutor("Test", daemon, queueSize);
      AtomicInteger numberOfThingsThatHappened = new AtomicInteger();
      executor.execute(() ->
      {
         LogTools.info("ints[0] = {}", ints[0]);
         LogTools.info("ints[1] = {}", ints[1]);
      },
      exception ->
      {
         MESSAGE_AND_TRACE_WITH_THREAD_NAME.handleException(exception);
         assertTrue(exception instanceof ArrayIndexOutOfBoundsException);
         numberOfThingsThatHappened.getAndIncrement();
      });

      executor.submit(() ->
      {
         LogTools.info("ints[0] = {}", ints[0]);
         LogTools.info("ints[1] = {}", ints[1]);
      },
      exception ->
      {
         MESSAGE_AND_TRACE_WITH_THREAD_NAME.handleException(exception);
         assertTrue(exception instanceof ArrayIndexOutOfBoundsException);
         numberOfThingsThatHappened.getAndIncrement();
      });

      executor.submit(() ->
      {
         LogTools.info("ints[0] = {}", ints[0]);
         LogTools.info("ints[1] = {}", ints[1]);

         return 5;
      },
      (result, exception) ->
      {
         MESSAGE_AND_TRACE_WITH_THREAD_NAME.handleException(exception);
         assertTrue(exception instanceof ArrayIndexOutOfBoundsException);
         numberOfThingsThatHappened.getAndIncrement();
      });

      executor.submit(() ->
      {
         LogTools.info("ints[0] = {}", ints[0]);

         return 5;
      },
      (result, exception) ->
      {
         assertEquals(5, result);
         numberOfThingsThatHappened.getAndIncrement();
      });

      ThreadTools.sleepSeconds(0.2);

      assertEquals(4, numberOfThingsThatHappened.get());
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
