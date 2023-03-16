package us.ihmc.missionControl.monitor;

import us.ihmc.tools.thread.PausablePeriodicThread;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.concurrent.ConcurrentLinkedQueue;

public abstract class ProcessOutputMonitor
{
   private static final double DEFAULT_READ_PERIOD_SECONDS = 0.25;

   private final String[] command;
   private final PausablePeriodicThread thread;
   private volatile boolean running;

   public ProcessOutputMonitor(String... command)
   {
      this(DEFAULT_READ_PERIOD_SECONDS, command);
   }

   public ProcessOutputMonitor(double parsePeriodSeconds, String... command)
   {
      this.command = command;
      thread = new PausablePeriodicThread(getClass().getName() + "-Reader-Parser", parsePeriodSeconds, true, () ->
      {
         try
         {
            parse(read());
         }
         catch (Exception e)
         {
            e.printStackTrace();
         }
      });
   }

   public void start()
   {
      thread.start();
      running = true;
   }

   public void stop()
   {
      thread.stop();
      running = false;
   }

   // TODO: cleanup
   public String[] read() throws IOException, InterruptedException
   {
      ProcessBuilder processBuilder = new ProcessBuilder(command);

      Process process = processBuilder.start();
      BufferedReader bufferedReader = new BufferedReader(new InputStreamReader(process.getInputStream()));

      ConcurrentLinkedQueue<Integer> characterQueue = new ConcurrentLinkedQueue<>();

      int read;
      while ((read = bufferedReader.read()) > -1 && running)
      {
         characterQueue.add(read);
      }

      process.destroy();
      process.waitFor();

      final String[] lines;

      if (!characterQueue.isEmpty())
      {
         StringBuilder output = new StringBuilder();
         while (!characterQueue.isEmpty())
         {
            output.append((char) ((int) characterQueue.poll()));
         }

         String outputString = output.toString();
         lines = outputString.split("\n");
      }
      else
      {
         lines = new String[] {};
      }

      return lines;
   }

   public abstract void parse(String[] lines);
}
