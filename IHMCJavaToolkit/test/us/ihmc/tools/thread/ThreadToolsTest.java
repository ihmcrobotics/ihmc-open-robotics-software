package us.ihmc.tools.thread;

import static org.junit.Assert.assertEquals;

import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

import org.apache.commons.lang3.SystemUtils;
import org.junit.Assert;
import org.junit.Test;

import us.ihmc.tools.testing.TestPlanTarget;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class ThreadToolsTest
{
   /**
    * Tests giving stringed commands to runCommandLine
    * does not produce an error, which is the most likely
    * failure mode.
    */
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testRunCommandLineStringedCommmands()
   {
      if (SystemUtils.IS_OS_WINDOWS)
      {
         ThreadTools.runCommandLine("dir & cd .. & dir");
      }
      else if (SystemUtils.IS_OS_MAC)
      {
         ThreadTools.runCommandLine("ls -a; cd .");
      }
      else if (SystemUtils.IS_OS_LINUX)
      {
         ThreadTools.runCommandLine("ls -a; cd .");
      }
      else
      {
         Assert.fail("Using unsupported OS");
      }
   }

   /**
    * Tests capturing the output of an echo.
    */
   @DeployableTestMethod(estimatedDuration = 0.1, targets = TestPlanTarget.Exclude)
   @Test(timeout = 30000)
   public void testRunCommandLineEchoOutput()
   {
      final StringBuilder commandLineOutput = new StringBuilder();

      PrintStream commandOutput = new PrintStream(new OutputStream()
      {
         @Override
         public void write(int b) throws IOException
         {
            commandLineOutput.append((char) b);
         }
      });

      ThreadTools.runCommandLine("echo Hi", commandOutput);

      Assert.assertTrue("Output not correct: " + commandLineOutput.toString(), commandLineOutput.toString().matches("Hi\\s*"));
   }

   @DeployableTestMethod(estimatedDuration = 0.1, targets = TestPlanTarget.Flaky)
   @Test(timeout = 30000)
   public void testTimeLimitScheduler()
   {
      final int ITERATIONS = 100;
      final double EPSILON = 5;
      
      TimeUnit timeUnit = TimeUnit.MILLISECONDS;
      long initialDelay = 0;
      long delay = 3;
      long timeLimit = 30;
      
      
      final Runnable runnable = new Runnable()
      
      {
         @Override
         public void run()
         {
            //Do some calculation
            Math.sqrt(Math.PI);
         }
      };
      
      for(int i = 0; i < ITERATIONS; i++)
      {
         long startTime = System.currentTimeMillis();
         ScheduledFuture<?> future = ThreadTools.scheduleWithFixeDelayAndTimeLimit(getClass().getSimpleName(), runnable, initialDelay, delay, timeUnit, timeLimit);
         while(!future.isDone());
         long endTime = System.currentTimeMillis();
         assertEquals(timeLimit, endTime - startTime, EPSILON);
      }
   }
   
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testIterationLimitScheduler()
   {
      TimeUnit timeUnit = TimeUnit.MILLISECONDS;
      long initialDelay = 0;
      long delay = 10;
      final int iterations = 10;
      
      final AtomicInteger counter = new AtomicInteger();
      
      final Runnable runnable = new Runnable()
      {
         @Override
         public void run()
         {
            counter.incrementAndGet();
         }
      };
      
      ScheduledFuture<?> future = ThreadTools.scheduleWithFixedDelayAndIterationLimit(getClass().getSimpleName(), runnable, initialDelay, delay, timeUnit, iterations);
      
      while(!future.isDone());
      assertEquals(iterations, counter.get());
   }
}
