package us.ihmc.tools.processManagement;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.log.LogTools;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.PrintStream;
import java.util.Timer;
import java.util.TimerTask;

public class ProcessStreamGobbler extends Thread
{
   private final Process process;
   private final PrintStream outputStream;
   private final InputStream inputStream;
   private final String processName;
   private final String processPrintingPrefix;
   private Timer currentTimer;

   private BufferedReader bufferedReader;

   private final Object monitor = new Object();
   private boolean shutdown = false;

   public ProcessStreamGobbler(final String processName, Process process, InputStream inputStream, PrintStream outputStream)
   {
      this(processName, process, inputStream, outputStream, "[Process: " + processName + "] ");
   }

   public ProcessStreamGobbler(final String processName, Process process, InputStream inputStream, PrintStream outputStream, String processPrintingPrefix)
   {
      super("ProcessStreamGobbler_" + processName);
      this.process = process;
      this.inputStream = inputStream;
      this.outputStream = outputStream;
      this.processName = processName;
      this.processPrintingPrefix = processPrintingPrefix;

      Runtime.getRuntime().addShutdownHook(new Thread(this::onJVMShutdown, "IHMC-ProcessStreamGobblerShutdown"));
   }

   @Override
   public void run()
   {
      try
      {
         InputStreamReader inputStreamReader = new InputStreamReader(inputStream);
         bufferedReader = new BufferedReader(inputStreamReader);
         while (!shutdown)
         {
            if (!process.isAlive())
            {
               LogTools.info("{} is no longer alive. Shutting down...", processName);
               startShutdown();
               break;
            }

            if (!bufferedReader.ready())
            {
               startTimer();

               synchronized (monitor)
               {
                  monitor.wait();
               }
            }

            while (bufferedReader.ready())
            {
               processStreams(bufferedReader);
            }
         }

         while (bufferedReader.ready())
         {
            processStreams(bufferedReader);
         }

         bufferedReader.close();
      }
      catch (IOException | InterruptedException e)
      {
         e.printStackTrace();
      }
   }

   private void startTimer()
   {
      currentTimer = new Timer("ProcessStreamGobbler_" + processName + "Timer");
      currentTimer.schedule(new TimerTask()
      {
         @Override
         public void run()
         {
            try
            {
               if (!process.isAlive())
               {
                  LogTools.info("{} is no longer alive. Shutting down...", processName);
                  startShutdown();
               }

               if (bufferedReader.ready())
               {
                  synchronized (monitor)
                  {
                     monitor.notify();
                  }
                  currentTimer.cancel();
                  currentTimer.purge();
               }
            }
            catch (IOException e)
            {
               e.printStackTrace();
            }
         }
      }, 0, 500);
   }

   private void processStreams(BufferedReader bufferedReader) throws IOException
   {
      try
      {
         String line = bufferedReader.readLine();
         if (line != null)
         {
            outputStream.println(processPrintingPrefix == null ? line : processPrintingPrefix + line);
            outputStream.flush();
         }
      }
      catch (IOException ioException)
      {
         if (ioException.getMessage().equals("Stream closed"))
         {
            LogTools.info("{}: Input stream closed. Shutting down...", processName);
            startShutdown();
         }
         else
         {
            throw ioException;
         }
      }
   }

   private void onJVMShutdown()
   {
      currentTimer.cancel();
      currentTimer.purge();
      interrupt();
      ExceptionTools.handle(() -> bufferedReader.close(), DefaultExceptionHandler.PRINT_MESSAGE);
   }

   public synchronized void startShutdown()
   {
      shutdown = true;
      currentTimer.cancel();
      currentTimer.purge();
      synchronized (monitor)
      {
         monitor.notify();
      }
   }
}