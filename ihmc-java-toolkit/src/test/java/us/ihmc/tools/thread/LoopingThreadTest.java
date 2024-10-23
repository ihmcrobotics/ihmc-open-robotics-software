package us.ihmc.tools.thread;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.tools.time.FrequencyCalculator;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.jupiter.api.Assertions.*;

public class LoopingThreadTest
{
   private static final String name = "TestLoopingThread";

   @Test
   public void testStartClose()
   {
      LoopingThread thread = new LoopingThread(() ->
      {
         System.out.println("Test Thread Running");
         Thread.sleep(500);
      }, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, name);

      thread.start();
      assertTrue(thread.isLooping());
      assertTrue(thread.isAlive());

      thread.close();
      try
      {
         Thread.sleep(1000);
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }
      assertFalse(thread.isLooping());
      assertFalse(thread.isAlive());
   }

   @Test
   public void testStartPauseStart()
   {
      LoopingThread thread = new LoopingThread(() ->
      {
         System.out.println("Test Thread Running");
         Thread.sleep(500);
      }, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, name);

      thread.start();
      assertTrue(thread.isLooping());
      assertTrue(thread.isAlive());

      thread.pause();
      try
      {
         Thread.sleep(1000);
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }
      assertFalse(thread.isLooping());
      assertTrue(thread.isAlive());

      thread.start();
      assertTrue(thread.isLooping());
      assertTrue(thread.isAlive());

      thread.close();
      try
      {
         Thread.sleep(1000);
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }
      assertFalse(thread.isLooping());
      assertFalse(thread.isAlive());
   }

   @Test
   public void testDoubleStart()
   {
      LoopingThread thread = new LoopingThread(() ->
      {
         System.out.println("Test Thread Running");
         Thread.sleep(500);
      }, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, name);

      thread.start();
      assertTrue(thread.isLooping());
      assertTrue(thread.isAlive());

      thread.start();
      assertTrue(thread.isLooping());
      assertTrue(thread.isAlive());

      thread.close();
      try
      {
         Thread.sleep(1000);
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }
      assertFalse(thread.isLooping());
      assertFalse(thread.isAlive());
   }

   @Test
   public void testDoubleClose()
   {
      LoopingThread thread = new LoopingThread(() ->
      {
         System.out.println("Test Thread Running");
         Thread.sleep(500);
      }, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, name);

      thread.start();
      assertTrue(thread.isLooping());
      assertTrue(thread.isAlive());

      thread.close();
      try
      {
         Thread.sleep(1000);
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }
      assertFalse(thread.isLooping());
      assertFalse(thread.isAlive());

      thread.close();
      try
      {
         Thread.sleep(1000);
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }
      assertFalse(thread.isLooping());
      assertFalse(thread.isAlive());
   }

   @Test
   public void testCloseWithoutStart()
   {
      LoopingThread thread = new LoopingThread(() ->
      {
         System.out.println("Test Thread Running");
         Thread.sleep(500);
      }, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, name);

      thread.close();
      assertFalse(thread.isLooping());
      assertFalse(thread.isAlive());
   }

   @Test
   public void testBlockingClose()
   {
      LoopingThread thread = new LoopingThread(() ->
      {
         System.out.println("Test Thread Running");
         Thread.sleep(500);
      }, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, name);

      thread.start();
      assertTrue(thread.isLooping());
      assertTrue(thread.isAlive());

      thread.blockingClose();
      assertFalse(thread.isLooping());
      assertFalse(thread.isAlive());
   }

   @Test
   public void testRunOnce()
   {
      AtomicInteger runCounter = new AtomicInteger(0);
      LoopingThread thread = new LoopingThread(() ->
      {
         assert runCounter.incrementAndGet() == 1;
      }, name);

      thread.runOnce();
      ThreadTools.sleep(500);
      thread.blockingClose();
      assertEquals(1, runCounter.get());
      assertFalse(thread.isLooping());
      assertFalse(thread.isAlive());
   }

   @Test
   public void testOverride()
   {
      AtomicBoolean ranSuccessfully = new AtomicBoolean(false);

      LoopingThread thread = new LoopingThread()
      {
         @Override
         public void runInLoop()
         {
            ranSuccessfully.set(true);
         }
      };

      thread.start();
      ThreadTools.sleep(500);
      thread.blockingClose();
      assertTrue(ranSuccessfully.get());
      assertFalse(thread.isLooping());
      assertFalse(thread.isAlive());
   }

   @Test
   public void testLoopFrequencyLimit()
   {
      FrequencyCalculator frequencyCalculator = new FrequencyCalculator();

      double targetFrequency = 5.0;
      LoopingThread thread = new LoopingThread(frequencyCalculator::ping, targetFrequency, name);

      thread.start();
      ThreadTools.sleep(3000);
      assertEquals(targetFrequency, frequencyCalculator.getFrequency(), 0.1);

      targetFrequency = 30.0;
      thread.limitLoopFrequency(targetFrequency);
      ThreadTools.sleep(3000);
      assertEquals(targetFrequency, frequencyCalculator.getFrequency(), 0.1);

      thread.blockingClose();
   }
}
