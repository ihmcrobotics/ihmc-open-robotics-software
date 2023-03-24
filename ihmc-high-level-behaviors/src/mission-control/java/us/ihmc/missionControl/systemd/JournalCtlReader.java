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

   public JournalCtlReader(String serviceName, Consumer<List<String>> logConsumer)
   {
      this.serviceName = serviceName;
      this.logConsumer = logConsumer;

      PausablePeriodicThread readerThread = new PausablePeriodicThread("journalctl-processor", 0.2, true, this::processOutput);
      readerThread.start();
      start();
   }

   public void start()
   {
      ThreadTools.startAsDaemon(() ->
      {
         ProcessBuilder processBuilder = new ProcessBuilder("journalctl", "-ef", "-u", serviceName);

         try
         {
            Process process = processBuilder.start();
            BufferedReader reader = new BufferedReader(new InputStreamReader(process.getInputStream()));
            String line;
            while ((line = reader.readLine()) != null)
            {
               System.out.println(line);
            }
            reader.close();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }, "journalctl-reader-" + serviceName);
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
