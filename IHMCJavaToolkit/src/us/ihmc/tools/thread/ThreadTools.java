package us.ihmc.tools.thread;

import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

import org.apache.commons.io.IOUtils;
import org.apache.commons.lang3.SystemUtils;

import us.ihmc.tools.io.StreamGobbler;

public class ThreadTools
{
   public static final int REASONABLE_WAITING_SLEEP_DURATION_MS = 10;

   public static void sleepSeconds(double secondsToSleep)
   {
      try
      {
         Thread.sleep((long) (secondsToSleep * 1000));
      }
      catch (InterruptedException e)
      {
      }
   }
   
   public static void sleep(long milliseconds)
   {
      try
      {
         Thread.sleep(milliseconds);
      }
      catch (InterruptedException ex)
      {
      }
   }
   
   public static void sleep(int milliseconds, int nanoseconds)
   {
      try
      {
         Thread.sleep(milliseconds, nanoseconds);
      }
      catch (InterruptedException ex)
      {
      }
   }

   public static void sleepForever()
   {
      while (true)
      {
         ThreadTools.sleep(1000);
      }
   }
   
   /**
    * Runs commands in your native command line environment.
    * <p>
    * This method supports Linux, Mac, and Windows with different behavior for each. Redirects console stdout
    * and stderr to System.out. Note on Windows: Use & to string commands together. ex. 'dir & cd .. & dir'
    * </p>
    * <p>
    * Fails if command is not valid, /bin/bash does not exist, or using a weird OS.
    * </p>
    * 
    * @param commandLine  command to run (Unix ex. "ls -a; nano a.txt") (Windows ex. "dir & cd .. & dir")
    * @param commandOutput  command output will be copied to this steam upon completion
    * @return The process that was started, waited for, and whose output was copied to System.out.
    * Returns when command finishes
    */
   public static void runCommandLine(String commandLine, OutputStream commandOutput)
   {    
      String[] commands = null;
      
      if(SystemUtils.IS_OS_WINDOWS)
      {
         commands = new String[] {"cmd.exe", "/c", commandLine};
      }
      else if (SystemUtils.IS_OS_MAC)
      {
         commands = new String[] {"/bin/bash", "-l", "-c", commandLine}; // TODO Fix me
      }
      else if (SystemUtils.IS_OS_LINUX)
      {
         commands = new String[] {"/bin/bash", "-l", "-c", commandLine};
      }
      else
      {
         return;
      }
      
      ProcessBuilder processBuilder = new ProcessBuilder(commands);
      if (commandOutput == System.out)
         processBuilder.inheritIO();
      processBuilder.redirectErrorStream(true);

      try
      {
         Process process = processBuilder.start();
         process.waitFor();
         IOUtils.copy(process.getInputStream(), commandOutput);
      }
      catch (IOException | InterruptedException e)
      {
         e.printStackTrace();
      }
   }
   
   /**
    * Runs commands in your native command line environment.
    * 
    * @see {@link #runCommandLine(String, OutputStream)}
    */
   public static void runCommandLine(String commandLine)
   {
      runCommandLine(commandLine, System.out);
   }

   // the following method and class were jacked from:
   // http://www.javaworld.com/javaworld/jw-12-2000/jw-1229-traps.html?page=3
   // thank you javaworld, you've changed my life!
   public static Process runCommand(String command, PrintStream outputStream, PrintStream errorStream)
   {
      try
      {
         Runtime runtime = Runtime.getRuntime();
         System.out.println("Execing " + command);
         Process process = runtime.exec(command);

         // any error message?
         StreamGobbler errorGobbler = new StreamGobbler(process.getErrorStream(), errorStream);

         // any output?
         StreamGobbler outputGobbler = new StreamGobbler(process.getInputStream(), outputStream);

         outputGobbler.setDaemon(true);
         errorGobbler.setDaemon(true);

         // kick them off
         errorGobbler.start();
         outputGobbler.start();

         return process;

         //
         //// any error???
         // int exitVal = proc.waitFor();
         // System.out.println("ExitValue: " + exitVal);
      }
      catch (Throwable t)
      {
         t.printStackTrace();
      }

      return null;
   }

   public static void startAsDaemon(Runnable daemonThreadRunnable, String threadName)
   {
      Thread daemonThread = new Thread(daemonThreadRunnable, threadName);
      daemonThread.setDaemon(true);
      daemonThread.start();
   }

   public static void waitUntilNextMultipleOf(long waitMultipleMS) throws InterruptedException
   {
      waitUntilNextMultipleOf(waitMultipleMS, 0);
   }

   public static void waitUntilNextMultipleOf(long waitMultipleMS, long moduloOffset) throws InterruptedException
   {
      long startTime = System.currentTimeMillis();
      long numberOfMultiplesThusFar = (startTime - moduloOffset) / waitMultipleMS;
      long endTime = (numberOfMultiplesThusFar + 1) * waitMultipleMS + moduloOffset;
      waitUntil(endTime);
   }

   public static void waitUntil(long endTime) throws InterruptedException
   {
      while (true)
      {
         if (endTime <= System.currentTimeMillis())
            break;
         Thread.sleep(REASONABLE_WAITING_SLEEP_DURATION_MS);
      }
   }

   
   public static ThreadFactory getNamedThreadFactory(final String name)
   {
      return new ThreadFactory()
      {
         private final AtomicInteger threadNumber = new AtomicInteger(1);

         public Thread newThread(Runnable r)
         {
            Thread t = new Thread(r, name + "-thread-" + threadNumber.getAndIncrement());

            if (t.isDaemon())
               t.setDaemon(false);
            if (t.getPriority() != Thread.NORM_PRIORITY)
               t.setPriority(Thread.NORM_PRIORITY);

            return t;
         }
      };
   }
   
   public static String getBaseClassName()
   {
      StackTraceElement[] stack = Thread.currentThread().getStackTrace();
      String className = stack[stack.length - 1].getClassName();
      return className;
   }
   
   public static String getBaseSimpleClassName()
   {
      String baseClassName = getBaseClassName();
      int lastDotIndex = baseClassName.lastIndexOf('.');
      String simpleClassName = baseClassName.substring(lastDotIndex + 1);
      return simpleClassName;
   }

   public static void interruptLiveThreadsExceptThisOneContaining(String stringToContain)
   {
      Map<Thread, StackTraceElement[]> allStackTraces = Thread.getAllStackTraces();
      Set<Thread> threadSet = allStackTraces.keySet();
      
      for (Thread thread : threadSet)
      {
         if (thread.isAlive() && thread != Thread.currentThread())
         {
            if (thread.getName().contains(stringToContain))
            {
//               System.out.println("Interrupting thread " + thread.getName());
               thread.interrupt();
            }
         }
      }
   }
   
   public static void interruptAllAliveThreadsExceptThisOne()
   {
      Map<Thread, StackTraceElement[]> allStackTraces = Thread.getAllStackTraces();
      Set<Thread> threadSet = allStackTraces.keySet();
      
      for (Thread thread : threadSet)
      {
         if (thread.isAlive() && thread != Thread.currentThread())
         {
            thread.interrupt();
         }
      }
   }

   public static ScheduledFuture<?> scheduleWithFixeDelayAndTimeLimit(String threadName, final Runnable runnable, long initialDelay, long delay, TimeUnit timeUnit, final long timeLimit)
   {
      ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor(getNamedThreadFactory(threadName));
      final ScheduledFuture<?> handle = scheduler.scheduleWithFixedDelay(runnable, initialDelay, delay, timeUnit);
      ScheduledFuture<?> handleKiller = scheduler.schedule(new Runnable()
      {
         @Override
         public void run()
         {
            handle.cancel(true);
         }
      }, timeLimit, timeUnit);
      
      return handleKiller;
   }

   public static ScheduledFuture<?> scheduleWithFixedDelayAndIterationLimit(String threadName, final Runnable runnable, long initialDelay, final long delay, final TimeUnit timeUnit, final int iterations)
   {
      final AtomicInteger counter = new AtomicInteger();
      ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(2, getNamedThreadFactory(threadName));
      final ScheduledFuture<?> handle = scheduler.scheduleWithFixedDelay(new Runnable()
      {
         @Override
         public void run()
         {
            if(counter.get() < iterations)
            {
               runnable.run();
               counter.incrementAndGet();
            }
         }
      }, initialDelay, delay, timeUnit);
      
      ScheduledFuture<?> handleKiller = scheduler.schedule(new Runnable()
      {
         @Override
         public void run()
         {
            while(counter.get() < iterations)
            {
               sleep(TimeUnit.MILLISECONDS.convert(delay, timeUnit));
            }
            handle.cancel(true);
         }
      }, 0, timeUnit);
      
      return handleKiller;
   }

}
