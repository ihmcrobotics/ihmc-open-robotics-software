package us.ihmc.tools.thread;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.tools.time.FrequencyCalculator;

import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.jupiter.api.Assertions.*;

public class PausableLoopingThreadTest
{
   private static final String NAME = "TestLoopingThread";

   @Test
   public void testStartDestroy()
   {
      PausableLoopingThread thread = new PausableLoopingThread(() ->
      {
         System.out.println("Test Thread Running");
         Thread.sleep(500);
      }, NAME);

      thread.start();
      assertTrue(thread.isLooping());
      assertTrue(thread.isAlive());

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
      assertFalse(thread.isAlive());
   }

   @Test
   public void testStartPauseStart()
   {
      PausableLoopingThread thread = new PausableLoopingThread(() ->
      {
         System.out.println("Test Thread Running");
         Thread.sleep(500);
      }, NAME);

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
      assertFalse(thread.isAlive());
   }

   @Test
   public void testDoubleStart()
   {
      PausableLoopingThread thread = new PausableLoopingThread(() ->
      {
         System.out.println("Test Thread Running");
         Thread.sleep(500);
      }, NAME);

      thread.start();
      assertTrue(thread.isLooping());
      assertTrue(thread.isAlive());

      thread.start();
      assertTrue(thread.isLooping());
      assertTrue(thread.isAlive());

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
      assertFalse(thread.isAlive());
   }

   @Test
   public void testDoubleDestroy()
   {
      PausableLoopingThread thread = new PausableLoopingThread(() ->
      {
         System.out.println("Test Thread Running");
         Thread.sleep(500);
      }, NAME);

      thread.start();
      assertTrue(thread.isLooping());
      assertTrue(thread.isAlive());

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
      assertFalse(thread.isAlive());

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
      assertFalse(thread.isAlive());
   }

   @Test
   public void testDestroyWithoutStart()
   {
      PausableLoopingThread thread = new PausableLoopingThread(() ->
      {
         System.out.println("Test Thread Running");
         Thread.sleep(500);
      }, NAME);

      thread.blockingDestroy();
      assertFalse(thread.isLooping());
      assertFalse(thread.isAlive());
   }

   @Test
   public void testBlockingDestroy()
   {
      PausableLoopingThread thread = new PausableLoopingThread(() ->
      {
         System.out.println("Test Thread Running");
         Thread.sleep(500);
      }, NAME);

      thread.start();
      assertTrue(thread.isLooping());
      assertTrue(thread.isAlive());

      thread.blockingDestroy();
      assertFalse(thread.isLooping());
      assertFalse(thread.isAlive());
   }

   @Test
   public void testLoopOnce()
   {
      AtomicInteger loopCounter = new AtomicInteger(0);
      PausableLoopingThread thread = new PausableLoopingThread(() ->
      {
         assert loopCounter.incrementAndGet() == 1;
      }, NAME);

      thread.loopOnce();
      ThreadTools.sleep(500);
      thread.blockingDestroy();
      assertEquals(1, loopCounter.get());
      assertFalse(thread.isLooping());
      assertFalse(thread.isAlive());
   }

   @Test
   public void testLoopNIterations()
   {
      AtomicInteger loopCounter = new AtomicInteger(0);
      Notification loopedNotification = new Notification();
      Notification loopAssertedNotification = new Notification();

      PausableLoopingThread thread = new PausableLoopingThread(() ->
      {
         loopCounter.set(loopCounter.get() + 1);
         loopedNotification.set();
         loopAssertedNotification.blockingPoll();
      }, NAME);

      for (int targetLoops = 1; targetLoops < 25; ++targetLoops)
      {
         loopCounter.set(0);
         thread.loopNIterations(targetLoops);
         for (int i = 0; i < targetLoops; ++i)
         {
            loopedNotification.blockingPoll();
            assertEquals(i + 1, loopCounter.get());
            loopAssertedNotification.set();
         }
      }
   }

   @Test
   public void testAddIterations()
   {
      AtomicInteger loopCounter = new AtomicInteger(0);
      PausableLoopingThread thread = new PausableLoopingThread(loopCounter::getAndIncrement, NAME);

      int add = 20;
      int subtract = -10;
      int increment = 1;
      int total = add + subtract + increment;

      thread.addIterations(add);
      thread.addIterations(subtract);
      thread.incrementIterations();
      ThreadTools.sleep(500);
      thread.blockingDestroy();
      assertEquals(total, loopCounter.get());
   }

   @Test
   public void testLoopFrequencyLimit()
   {
      FrequencyCalculator frequencyCalculator = new FrequencyCalculator();

      double targetFrequency = 5.0;
      PausableLoopingThread thread = new PausableLoopingThread(frequencyCalculator::ping, targetFrequency, NAME);

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
      }, NAME);

      // Test during free spin
      thread.start();
      for (int i = 0; i < 100; ++i)
      {
         thread.interrupt();
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
         thread.interrupt();
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
         thread.interrupt();
         synchronized (interruptCount)
         {
            interruptCount.wait(500);
         }
         assertEquals(i + 1, interruptCount.get());
      }
      thread.blockingDestroy();
   }

   @Test
   public void testOverride()
   {
      AtomicInteger loopCounter = new AtomicInteger(0);
      PausableLoopingThread thread = new PausableLoopingThread(NAME)
      {
         @Override
         protected void runInLoop()
         {
            loopCounter.set(loopCounter.get() + 1);
         }
      };

      int targetLoops = 15;
      thread.loopNIterations(targetLoops);
      ThreadTools.sleep(500);
      thread.blockingDestroy();
      assertEquals(targetLoops, loopCounter.get());
   }
}
