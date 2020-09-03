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
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;

/**
 * @author Igor Kalkov <a href="mailto:ikalkov@ihmc.us">(ikalkov@ihmc.us)</a>
 */
public abstract class ProcessSpawner
{
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
      Runtime.getRuntime().addShutdownHook(new Thread(this::shutdown, "IHMC-ProcessSpawnerShutdown"));
   }

   protected Process spawn(String name,
                           String[] spawnString,
                           ProcessBuilder builder,
                           File outputFile,
                           File errorFile,
                           ExitListener exitListener)
   {
      return spawn(name, spawnString, builder, outputFile, errorFile, null, null, defaultPrintingPrefix(name), exitListener);
   }

   protected Process spawn(String name,
                           String[] spawnString,
                           ProcessBuilder builder,
                           PrintStream outputStream,
                           PrintStream errorStream,
                           ExitListener exitListener)
   {
      return spawn(name, spawnString, builder, null, null, outputStream, errorStream, defaultPrintingPrefix(name), exitListener);
   }

   protected Process spawn(String name,
                           String[] spawnString,
                           ProcessBuilder builder,
                           File outputFile,
                           File errorFile,
                           PrintStream outputStream,
                           PrintStream errorStream,
                           String processPrintingPrefix,
                           ExitListener exitListener)
   {
      if (outputFile != null)
      {
         builder.redirectOutput(outputFile);
      }

      if (errorFile != null)
      {
         builder.redirectError(errorFile);
      }

      try
      {
         LogTools.trace("Forking process: {}{}", System.getProperty("line.separator"), Arrays.toString(spawnString));
         Process process = builder.start();
         ImmutablePair<Process, String> newPair = new ImmutablePair<>(process, name);
         processes.add(newPair);

         setProcessExitListener(process, exitListener);

         if (outputFile == null)
         {
            ProcessStreamGobbler processStreamGobbler = new ProcessStreamGobbler(name,
                                                                                 process,
                                                                                 process.getInputStream(),
                                                                                 outputStream == null ? System.out : outputStream,
                                                                                 processPrintingPrefix);
            streamGobblers.add(processStreamGobbler);
            processStreamGobbler.start();
         }

         if (errorFile == null)
         {
            ProcessStreamGobbler processStreamGobbler = new ProcessStreamGobbler(name,
                                                                                 process,
                                                                                 process.getErrorStream(),
                                                                                 errorStream == null ? System.err : errorStream,
                                                                                 processPrintingPrefix);
            streamGobblers.add(processStreamGobbler);
            processStreamGobbler.start();
         }

         asyncWaitForExit(newPair);
         return process;
      }
      catch (IOException e)
      {
         reportProcessSpawnException(errorFile, errorStream, e);
         return null;
      }
   }

   private void asyncWaitForExit(final ImmutablePair<Process, String> pair)
   {
      ThreadTools.startAThread(() ->
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
      }, "ProcessExitListener" + pair.getRight());
   }

   private void reportProcessSpawnException(File errorFile, PrintStream errorStream, IOException exception)
   {
      if (errorStream != null)
      {
         exception.printStackTrace(errorStream);
      }

      if (errorFile != null)
      {
         try
         {
            PrintStream ps = new PrintStream(errorFile);
            exception.printStackTrace(ps);
            ps.flush();
            ps.close();
         }
         catch (FileNotFoundException e)
         {
            e.printStackTrace();
         }
      }
   }

   public static String defaultPrintingPrefix(String commandString)
   {
      return "[Process: " + commandString + "] ";
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
