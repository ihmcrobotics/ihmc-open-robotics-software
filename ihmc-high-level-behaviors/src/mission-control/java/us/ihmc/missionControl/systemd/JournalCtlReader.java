package us.ihmc.missionControl.systemd;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.tools.thread.PausablePeriodicThread;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Consumer;

public class JournalCtlReader
{
   private final String serviceName;
   private final Consumer<List<String>> logConsumer;
   private final ConcurrentLinkedQueue<String> logLineQueue = new ConcurrentLinkedQueue<>();
   private final PausablePeriodicThread readerThread = new PausablePeriodicThread("journalctl-processor", 0.2, true, this::processOutput);

   private volatile boolean running = false;

   public JournalCtlReader(String serviceName, Consumer<List<String>> logConsumer)
   {
      this.serviceName = serviceName;
      this.logConsumer = logConsumer;
   }

   public void start()
   {
      ThreadTools.startAsDaemon(() ->
      {
         ProcessBuilder processBuilder = new ProcessBuilder("sudo", "journalctl", "--since", "1 hour ago", "-ef", "-u", serviceName);

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
            linesToSend.add(logLineQueue.poll());
         }

         logConsumer.accept(linesToSend);
      }
   }
}
