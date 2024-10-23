package us.ihmc.tools.thread;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.tools.time.FrequencyCalculator;

import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.jupiter.api.Assertions.*;

public class PausableLoopingThreadTest
{
   private static final String name = "TestLoopingThread";

   @Test
   public void testStartDestroy()
   {
      PausableLoopingThread thread = new PausableLoopingThread(() ->
      {
         System.out.println("Test Thread Running");
         Thread.sleep(500);
      }, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, name);

      thread.start();
      assertTrue(thread.isLooping());
      assertTrue(thread.getInternalThread().isAlive());

      thread.destroy();
      try
      {
         Thread.sleep(1000);
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }
      assertFalse(thread.isLooping());
      assertFalse(thread.getInternalThread().isAlive());
   }

   @Test
   public void testStartPauseStart()
   {
      PausableLoopingThread thread = new PausableLoopingThread(() ->
      {
         System.out.println("Test Thread Running");
         Thread.sleep(500);
      }, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, name);

      thread.start();
      assertTrue(thread.isLooping());
      assertTrue(thread.getInternalThread().isAlive());

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
      assertTrue(thread.getInternalThread().isAlive());

      thread.start();
      assertTrue(thread.isLooping());
      assertTrue(thread.getInternalThread().isAlive());

      thread.destroy();
      try
      {
         Thread.sleep(1000);
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }
      assertFalse(thread.isLooping());
      assertFalse(thread.getInternalThread().isAlive());
   }

   @Test
   public void testDoubleStart()
   {
      PausableLoopingThread thread = new PausableLoopingThread(() ->
      {
         System.out.println("Test Thread Running");
         Thread.sleep(500);
      }, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, name);

      thread.start();
      assertTrue(thread.isLooping());
      assertTrue(thread.getInternalThread().isAlive());

      thread.start();
      assertTrue(thread.isLooping());
      assertTrue(thread.getInternalThread().isAlive());

      thread.destroy();
      try
      {
         Thread.sleep(1000);
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }
      assertFalse(thread.isLooping());
      assertFalse(thread.getInternalThread().isAlive());
   }

   @Test
   public void testDoubleDestroy()
   {
      PausableLoopingThread thread = new PausableLoopingThread(() ->
      {
         System.out.println("Test Thread Running");
         Thread.sleep(500);
      }, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, name);

      thread.start();
      assertTrue(thread.isLooping());
      assertTrue(thread.getInternalThread().isAlive());

      thread.destroy();
      try
      {
         Thread.sleep(1000);
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }
      assertFalse(thread.isLooping());
      assertFalse(thread.getInternalThread().isAlive());

      thread.destroy();
      try
      {
         Thread.sleep(1000);
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }
      assertFalse(thread.isLooping());
      assertFalse(thread.getInternalThread().isAlive());
   }

   @Test
   public void testDestroyWithoutStart()
   {
      PausableLoopingThread thread = new PausableLoopingThread(() ->
      {
         System.out.println("Test Thread Running");
         Thread.sleep(500);
      }, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, name);

      thread.blockingDestroy();
      assertFalse(thread.isLooping());
      assertFalse(thread.getInternalThread().isAlive());
   }

   @Test
   public void testBlockingDestroy()
   {
      PausableLoopingThread thread = new PausableLoopingThread(() ->
      {
         System.out.println("Test Thread Running");
         Thread.sleep(500);
      }, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, name);

      thread.start();
      assertTrue(thread.isLooping());
      assertTrue(thread.getInternalThread().isAlive());

      thread.blockingDestroy();
      assertFalse(thread.isLooping());
      assertFalse(thread.getInternalThread().isAlive());
   }

   @Test
   public void testRunOnce()
   {
      AtomicInteger runCounter = new AtomicInteger(0);
      PausableLoopingThread thread = new PausableLoopingThread(() ->
      {
         assert runCounter.incrementAndGet() == 1;
      }, name);

      thread.runOnce();
      ThreadTools.sleep(500);
      thread.blockingDestroy();
      assertEquals(1, runCounter.get());
      assertFalse(thread.isLooping());
      assertFalse(thread.getInternalThread().isAlive());
   }

   @Test
   public void testLoopFrequencyLimit()
   {
      FrequencyCalculator frequencyCalculator = new FrequencyCalculator();

      double targetFrequency = 5.0;
      PausableLoopingThread thread = new PausableLoopingThread(frequencyCalculator::ping, targetFrequency, name);

      thread.start();
      ThreadTools.sleep(5000);
      assertEquals(targetFrequency, frequencyCalculator.getFrequency(), 0.1);

      targetFrequency = 30.0;
      thread.limitLoopFrequency(targetFrequency);
      ThreadTools.sleep(5000);
      assertEquals(targetFrequency, frequencyCalculator.getFrequency(), 0.1);

      thread.blockingDestroy();
   }

   @Test
   public void testInterrupt() throws InterruptedException
   {
      AtomicInteger interruptCount = new AtomicInteger(0);
      PausableLoopingThread thread = new PausableLoopingThread(() ->
      {
         try
         {
            Thread.sleep(10);
         }
         catch (InterruptedException interruptedException)
         {
            synchronized (interruptCount)
            {
               interruptCount.incrementAndGet();
               interruptCount.notify();
            }
         }
      }, name);

      // Test during free spin
      thread.start();
      for (int i = 0; i < 100; ++i)
      {
         thread.getInternalThread().interrupt();
         synchronized (interruptCount)
         {
            interruptCount.wait(500);
         }
         assertEquals(i + 1, interruptCount.get());
      }

      // Test during throttled looping
      interruptCount.set(0);
      thread.limitLoopFrequency(5.0);
      for (int i = 0; i < 50; ++i)
      {
         thread.getInternalThread().interrupt();
         synchronized (interruptCount)
         {
            interruptCount.wait(500);
         }
         assertEquals(i + 1, interruptCount.get());
      }

      // Test during pause
      interruptCount.set(0);
      thread.pause();
      for (int i = 0; i < 100; ++i)
      {
         thread.getInternalThread().interrupt();
         synchronized (interruptCount)
         {
            interruptCount.wait(500);
         }
         assertEquals(i + 1, interruptCount.get());
      }
      thread.blockingDestroy();
   }
}
