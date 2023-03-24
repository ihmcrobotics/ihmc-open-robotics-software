package us.ihmc.missionControl.systemd;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.tools.thread.PausablePeriodicThread;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Consumer;

public class JournalCtlReader
{
   private final String serviceName;
   private final Consumer<List<String>> logConsumer;
   private final ConcurrentLinkedQueue<Integer> characterQueue = new ConcurrentLinkedQueue<>();

   private static volatile boolean running = true;

   public JournalCtlReader(String serviceName, Consumer<List<String>> logConsumer)
   {
      this.serviceName = serviceName;
      this.logConsumer = logConsumer;

      PausablePeriodicThread readerThread = new PausablePeriodicThread("journalctl-processor", 0.2, true, this::processOutput);
      readerThread.start();
      start();
   }

   public void stop()
   {
      running = false;
   }

   public void start()
   {
      ThreadTools.startAsDaemon(() ->
      {
         ProcessBuilder processBuilder = new ProcessBuilder("journalctl", "-ef", "-u", serviceName);

         try
         {
            Process process = processBuilder.start();
            BufferedReader bufferedReader = new BufferedReader(new InputStreamReader(process.getInputStream()));

            int read;
            while ((read = bufferedReader.read()) > -1 && running)
            {
               characterQueue.add(read);
            }

            process.destroy();
            process.waitFor();
         }
         catch (IOException | InterruptedException e)
         {
            throw new RuntimeException(e);
         }
      }, "journalctl-reader-" + serviceName);
   }

   private void processOutput()
   {
      if (!characterQueue.isEmpty())
      {
         StringBuilder output = new StringBuilder();
         while (!characterQueue.isEmpty())
         {
            output.append((char) ((int) characterQueue.poll()));
         }

         String outputString = output.toString();
         String[] lines = outputString.split("\n");

         logConsumer.accept(Arrays.asList(lines));
      }
   }

   public static void main(String[] args)
   {
      JournalCtlReader reader = new JournalCtlReader("httpd", strings -> {
         for (String string : strings)
         {
            System.out.println(string);
         }
      });

      reader.start();

      while (true)
      {
         ThreadTools.sleep(5000);
      }
   }

}
