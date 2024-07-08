package us.ihmc.tools.thread;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.log.LogTools;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService.MESSAGE_AND_TRACE_WITH_THREAD_NAME;

public class MissingThreadToolsTest
{
   @Test
   public void testSleepAtLeast()
   {
      assertTrue(conductSleepTest(0.0000000000001, false));
      assertTrue(conductSleepTest(0.5e-9, false));
      assertTrue(conductSleepTest(1e-9, false));
      assertTrue(conductSleepTest(0.1, false));
      assertTrue(conductSleepTest(0.0001, false));
      assertTrue(conductSleepTest(0.0000000005, false));
      assertTrue(conductSleepTest(1.1, false));
      assertTrue(conductSleepTest(2.0, false));

      assertTrue(conductSleepTest(0.0000000000001, true));
      assertTrue(conductSleepTest(0.5e-9, true));
      assertTrue(conductSleepTest(1e-9, true));
      assertTrue(conductSleepTest(0.1, true));
      assertTrue(conductSleepTest(0.0001, true));
      assertTrue(conductSleepTest(0.0000000005, true));
      assertTrue(conductSleepTest(1.1, true));
      assertTrue(conductSleepTest(2.0, true));
   }

   private boolean conductSleepTest(double sleepDuration, boolean atLeast)
   {
      double before = Conversions.nanosecondsToSeconds(System.nanoTime());

      if (atLeast)
         MissingThreadTools.sleepAtLeast(sleepDuration);
      else
         MissingThreadTools.sleep(sleepDuration);

      double after = Conversions.nanosecondsToSeconds(System.nanoTime());

      double overslept = (after - before) - sleepDuration;

      LogTools.info("Overslept %f ms".formatted(Conversions.secondsToMilliseconds(overslept)));

      assertTrue(overslept < 0.005); // Assert we don't oversleep more than 5 milliseconds -- typically a lot lower

      return overslept > 0.0;
   }

   @Test
   public void testSecondsDecomposition()
   {
      double seconds = 0.017453292519943295;
      long nanoseconds = (long) (seconds * 1e9);
      long milliseconds = nanoseconds / 1_000_000L;
      nanoseconds %= 1_000_000L;

      double seconds2 = milliseconds / 1000.0 + nanoseconds / 1e9;
      double remaining = seconds - seconds2;

      LogTools.info("Original:      %.40f".formatted(seconds));
      LogTools.info("Reconstructed: %.40f".formatted(seconds2));
      LogTools.info("Remaining: %.40f".formatted(remaining));
      LogTools.info("s %% 1e-9: %.40f".formatted(seconds % 1e-9));

      conductModTest(1.0);
      conductModTest(0.1);
      conductModTest(1239991.123);
      conductModTest(181.232381318381);
      conductModTest(0.0);
      conductModTest(1e-9);
      conductModTest(0.5e-9);

      LogTools.info("%d".formatted((long) 0.7));
      LogTools.info("%d".formatted((long) 0.3));
      LogTools.info("%d".formatted((long) 1.0));
      LogTools.info("%d".formatted((long) 0.0));

      assertEquals(seconds, seconds2, 1e-9);
   }

   private void conductModTest(double number)
   {
      double nanos = number % 1e-9;

      LogTools.info("%.16f: %.35f  %b".formatted(number, nanos, nanos > 1e-9));
   }

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

      executor.destroy();
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

      executor.destroy();
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

      executor.destroy();
   }

   @Test
   public void testCancellableScheduledTasks()
   {
      ScheduledExecutorService scheduler = ThreadTools.newSingleDaemonThreadScheduledExecutor("Test");

      StringBuilder output = new StringBuilder();

      ScheduledFuture<?> scheduledFuture1 = scheduler.schedule(() -> output.append("A"), 400, TimeUnit.MILLISECONDS);
      ThreadTools.sleep(200);
      scheduledFuture1.cancel(false);
      scheduler.schedule(() -> output.append("B"), 400, TimeUnit.MILLISECONDS);
      ThreadTools.sleep(600);
      ScheduledFuture<StringBuilder> scheduledFuture2 = scheduler.schedule(() -> output.append("C"), 400, TimeUnit.MILLISECONDS);
      ThreadTools.sleep(200);
      scheduledFuture2.cancel(false);
      ThreadTools.sleep(600);
      scheduler.schedule(() -> output.append("D"), 400, TimeUnit.MILLISECONDS);
      ThreadTools.sleep(600);

      scheduler.schedule(() -> ExceptionTools.handle(() ->
      {
         output.append("E");
         throw new NullPointerException();
      }, DefaultExceptionHandler.PRINT_MESSAGE), 400, TimeUnit.MILLISECONDS);
      ThreadTools.sleep(600);
      ScheduledFuture<StringBuilder> scheduledFuture3 = scheduler.schedule(() -> output.append("F"), 400, TimeUnit.MILLISECONDS);
      ThreadTools.sleep(200);
      scheduledFuture3.cancel(false);
      ThreadTools.sleep(600);

      String recordedOutput = output.toString();
      assertEquals("BDE", recordedOutput);
      LogTools.info(recordedOutput);

      scheduler.shutdown();
   }
}
