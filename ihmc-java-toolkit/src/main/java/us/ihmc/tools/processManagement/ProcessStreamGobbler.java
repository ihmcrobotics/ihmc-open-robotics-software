package us.ihmc.tools.processManagement;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.PrintStream;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.atomic.AtomicReference;

public class ProcessStreamGobbler extends Thread
{
   private final PrintStream outputStream;
   private final InputStream inputStream;
   private final String processName;
   private final AtomicReference<String> processPrintingPrefix = new AtomicReference<>();
   private Timer currentTimer;

   private BufferedReader bufferedReader;

   private final Object monitor = new Object();
   private boolean shutdown = false;

   public ProcessStreamGobbler(final String processName, InputStream inputStream, PrintStream outputStream)
   {
      this(processName, inputStream, outputStream, "[Process: " + processName + "] ");
   }

   public ProcessStreamGobbler(final String processName, InputStream inputStream, PrintStream outputStream, String processPrintingPrefix)
   {
      super("ProcessStreamGobbler_" + processName);
      this.inputStream = inputStream;
      this.outputStream = outputStream;
      this.processName = processName;
      this.processPrintingPrefix.set(processPrintingPrefix);
//      Runtime.getRuntime().addShutdownHook(new Thread(() -> { }, "IHMC-ProcessStreamGobblerShutdown"));
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
      String line;
      line = bufferedReader.readLine();
      if (line != null && outputStream != null)
      {
         String prefix = processPrintingPrefix.get();
         outputStream.println(prefix == null ? line : prefix + line);
         outputStream.flush();
      }
   }

   public synchronized void startShutdown()
   {
      this.shutdown = true;
      currentTimer.cancel();
      currentTimer.purge();
      synchronized (monitor)
      {
         monitor.notify();
      }
   }
}