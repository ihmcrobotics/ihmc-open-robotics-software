package us.ihmc.tools.thread;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.exception.DefaultExceptionHandler;

import static org.junit.jupiter.api.Assertions.*;

public class RestartableThreadTest
{
   private static final String name = "TestRestartableThread";

   @Test
   public void testStartStop()
   {
      RestartableThread thread = new RestartableThread(name, () ->
      {
         System.out.println("Test Thread Running");
         Thread.sleep(500);
      });

      thread.start();
      assertTrue(thread.isRunning());
      assertTrue(thread.isAlive());

      thread.stop();
      try
      {
         Thread.sleep(1000);
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }
      assertFalse(thread.isRunning());
      assertFalse(thread.isAlive());
   }

   @Test
   public void testStartStopStart()
   {
      RestartableThread thread = new RestartableThread(name, () ->
      {
         System.out.println("Test Thread Running");
         Thread.sleep(500);
      });

      thread.start();
      assertTrue(thread.isRunning());
      assertTrue(thread.isAlive());

      thread.stop();
      try
      {
         Thread.sleep(1000);
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }
      assertFalse(thread.isRunning());
      assertFalse(thread.isAlive());

      thread.start();
      assertTrue(thread.isRunning());
      assertTrue(thread.isAlive());

      thread.stop();
      try
      {
         Thread.sleep(1000);
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }
      assertFalse(thread.isRunning());
      assertFalse(thread.isAlive());
   }

   @Test
   public void testDoubleStart()
   {
      RestartableThread thread = new RestartableThread(name, () ->
      {
         System.out.println("Test Thread Running");
         Thread.sleep(500);
      });

      thread.start();
      assertTrue(thread.isRunning());
      assertTrue(thread.isAlive());

      thread.start();
      assertTrue(thread.isRunning());
      assertTrue(thread.isAlive());

      thread.stop();
      try
      {
         Thread.sleep(1000);
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }
      assertFalse(thread.isRunning());
      assertFalse(thread.isAlive());
   }

   @Test
   public void testDoubleStop()
   {
      RestartableThread thread = new RestartableThread(name, () ->
      {
         System.out.println("Test Thread Running");
         Thread.sleep(500);
      });

      thread.start();
      assertTrue(thread.isRunning());
      assertTrue(thread.isAlive());

      thread.stop();
      try
      {
         Thread.sleep(1000);
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }
      assertFalse(thread.isRunning());
      assertFalse(thread.isAlive());

      thread.stop();
      try
      {
         Thread.sleep(1000);
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }
      assertFalse(thread.isRunning());
      assertFalse(thread.isAlive());
   }

   @Test
   public void testStopWithoutStart()
   {
      RestartableThread thread = new RestartableThread(name, () ->
      {
         System.out.println("Test Thread Running");
         Thread.sleep(500);
      });

      thread.stop();
      assertFalse(thread.isRunning());
      assertFalse(thread.isAlive());
   }

   @Test
   public void testExceptionHandling()
   {
      RestartableThread thread = new RestartableThread(name, DefaultExceptionHandler.RUNTIME_EXCEPTION, () ->
      {
         throw new NullPointerException("This is a test");
      });

      try
      {
         thread.start();
         Thread.sleep(1000);
      }
      catch (Exception e)
      {
         // Got an exception. This is good!
         assertTrue(true);
      }
      finally
      {
         assertFalse(thread.isRunning());
         assertFalse(thread.isAlive());
      }
   }

   @Test
   public void testBlockingStop()
   {
      RestartableThread thread = new RestartableThread(name, () ->
      {
         System.out.println("Test Thread Running");
         Thread.sleep(500);
      });

      thread.start();
      assertTrue(thread.isRunning());
      assertTrue(thread.isAlive());

      thread.blockingStop();
      assertFalse(thread.isRunning());
      assertFalse(thread.isAlive());
   }
}
