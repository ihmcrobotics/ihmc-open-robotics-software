package us.ihmc.missionControl.systemd;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.tools.thread.PausablePeriodicThread;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Consumer;

public class JournalCtlReader
{
   private final String serviceName;
   private final Consumer<List<String>> logConsumer;
   private final ConcurrentLinkedQueue<String> logLineQueue = new ConcurrentLinkedQueue<>();
   private final PausablePeriodicThread readerThread = new PausablePeriodicThread("journalctl-processor", 0.2, true, this::processOutput);
   /**
    * Used for refreshes
    */
   private final Queue<String> logHistory = new LinkedList<>()
   {
      @Override
      public boolean add(String o) {
         boolean added = super.add(o);
         while (added && size() > 1000)
            super.remove();
         return added;
      }
   };

   private volatile boolean running = false;

   public JournalCtlReader(String serviceName, Consumer<List<String>> logConsumer)
   {
      this.serviceName = serviceName;
      this.logConsumer = logConsumer;
   }

   public Queue<String> getLogHistory()
   {
      return logHistory;
   }

   public void start()
   {
      ThreadTools.startAsDaemon(() ->
      {
         ProcessBuilder processBuilder = new ProcessBuilder("sudo", "journalctl", "-n", "1000", "-ef", "-u", serviceName);

         try
         {
            Process process = processBuilder.start();
            BufferedReader reader = new BufferedReader(new InputStreamReader(process.getInputStream()));
            String line;
            while ((line = reader.readLine()) != null && running)
            {
               logLineQueue.add(line);
            }
            reader.close();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }, "journalctl-reader-" + serviceName);

      readerThread.start();
      running = true;
   }

   public void stop()
   {
      readerThread.stop();
      running = false;
   }

   private void processOutput()
   {
      if (!logLineQueue.isEmpty())
      {
         List<String> linesToSend = new ArrayList<>();
         while (!logLineQueue.isEmpty())
         {
            String line = logLineQueue.poll();
            linesToSend.add(logLineQueue.poll());
            logHistory.add(line);
         }

         logConsumer.accept(linesToSend);
      }
   }
}
