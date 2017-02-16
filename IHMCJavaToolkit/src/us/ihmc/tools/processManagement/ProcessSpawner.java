package us.ihmc.tools.processManagement;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.apache.commons.lang3.tuple.ImmutablePair;

/**
 * @author Igor Kalkov <a href="mailto:ikalkov@ihmc.us">(ikalkov@ihmc.us)</a>
 */
public abstract class ProcessSpawner
{
   private static final boolean DEBUG = false;

   private final boolean killChildProcessesOnShutdown;
   private final List<ProcessStreamGobbler> streamGobblers;

   /*
    * These are all volatile because none of them are accessed from within the
    * Spawner's main thread, but their state is important for the management of
    * child processes and so we don't want the JIT compiler to make assumptions
    * about them (which it does if they aren't explicitly marked volatile).
    */

   private volatile Queue<ImmutablePair<Process, String>> processes = new ConcurrentLinkedQueue<>();
   private volatile Map<Process, ExitListener> exitListeners = new ConcurrentHashMap<>();

   public ProcessSpawner(boolean killChildProcessesOnShutdown)
   {
      this.killChildProcessesOnShutdown = killChildProcessesOnShutdown;
      this.streamGobblers = new ArrayList<>();

      setupShutdownHook();
   }

   private void setupShutdownHook()
   {
      Runtime.getRuntime().addShutdownHook(new Thread(new Runnable()
      {
         @Override
         public void run()
         {
            shutdown();
         }
      }));
   }

   private void redirectProcessOutput(String commandString, Process p, boolean shouldGobbleOutput, boolean shouldGobbleError)
   {
      if (shouldGobbleOutput)
      {
         ProcessStreamGobbler processStreamGobbler = new ProcessStreamGobbler(commandString, p.getInputStream(), System.out);
         streamGobblers.add(processStreamGobbler);
         processStreamGobbler.start();
      }

      if (shouldGobbleError)
      {
         ProcessStreamGobbler processStreamGobbler = new ProcessStreamGobbler(commandString, p.getErrorStream(), System.err);
         streamGobblers.add(processStreamGobbler);
         processStreamGobbler.start();
      }
   }

   private void asyncWaitForExit(final ImmutablePair<Process, String> pair)
   {
      new Thread(new Runnable()
      {
         @Override
         public void run()
         {
            try
            {
               Process process = pair.getLeft();
               process.waitFor();
               processes.remove(pair);

               ExitListener exitListener = exitListeners.get(process);
               if (exitListener != null)
               {
                  int exitValue = process.exitValue();
                  exitListener.exited(exitValue);
                  exitListeners.remove(process);
               }
            }
            catch (InterruptedException e)
            {
               e.printStackTrace();
            }

         }
      }, "ProcessExitListener" + pair.getRight()).start();
   }

   private void printSpawnStringInfo(String[] spawnString)
   {
      System.out.print("[Process Spawner]: Forking process:" + System.getProperty("line.separator") + "\t");
      System.out.print(Arrays.toString(spawnString));
   }

   protected Process spawn(String commandString, String[] spawnString, ProcessBuilder builder, File outputLog, File errorLog, ExitListener exitListener)
   {
      Process process = null;
      boolean shouldGobbleOutput = true;
      boolean shouldGobbleError = true;

      if (DEBUG)
      {
         printSpawnStringInfo(spawnString);
      }

      if (outputLog != null)
      {
         builder.redirectOutput(outputLog);
         shouldGobbleOutput = false;
      }

      if (errorLog != null)
      {
         builder.redirectError(errorLog);
         shouldGobbleError = false;
      }

      try
      {
         process = builder.start();
         ImmutablePair<Process, String> newPair = new ImmutablePair<>(process, commandString);
         processes.add(newPair);

         setProcessExitListener(process, exitListener);
         redirectProcessOutput(commandString, process, shouldGobbleOutput, shouldGobbleError);
         asyncWaitForExit(newPair);
         return process;
      }
      catch (IOException e)
      {
         reportProcessSpawnException(errorLog, e);
         return null;
      }
   }

   private void reportProcessSpawnException(File errorLog, IOException exception)
   {
      if (errorLog == null)
      {
         exception.printStackTrace();
         return;
      }

      try
      {
         PrintStream ps = new PrintStream(errorLog);
         exception.printStackTrace(ps);
         ps.flush();
         ps.close();
      }
      catch (FileNotFoundException e)
      {
         e.printStackTrace();
      }
   }

   public void setProcessExitListener(Process process, ExitListener exitListener)
   {
      if (exitListener != null)
      {
         exitListeners.put(process, exitListener);
      }
   }

   public boolean hasRunningProcesses()
   {
      return !processes.isEmpty();
   }

   public void shutdown()
   {
      if (killChildProcessesOnShutdown)
      {
         for (ImmutablePair<Process, String> p : processes) { kill(p.getLeft()); }

         processes.clear();
         exitListeners.clear();
      }

      for (ProcessStreamGobbler streamGobbler : streamGobblers)
      {
         streamGobbler.startShutdown();
      }

      streamGobblers.clear();
   }

   public abstract void kill(Process process);
}
