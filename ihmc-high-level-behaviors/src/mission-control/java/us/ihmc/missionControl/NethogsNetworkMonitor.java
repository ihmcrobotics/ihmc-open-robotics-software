package us.ihmc.missionControl;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.tools.thread.PausablePeriodicThread;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.concurrent.ConcurrentLinkedQueue;

public class NethogsNetworkMonitor
{
   private final PausablePeriodicThread nethogsReaderThread;
   private Process process;
   private double kilobytesPerSecondSentTemp = 0.0;
   private double kilobytesPerSecondReceivedTemp = 0.0;
   private double kilobytesPerSecondSent = 0.0;
   private double kilobytesPerSecondReceived = 0.0;
   private BufferedReader bufferedReader;
   private final ConcurrentLinkedQueue<Integer> characterQueue = new ConcurrentLinkedQueue<>();
   private final Object syncObject = new Object();

   public NethogsNetworkMonitor()
   {
      nethogsReaderThread = new PausablePeriodicThread("NethogsReader", 0.2, true, this::processNethogsOutput);
   }

   public void start()
   {
      ThreadTools.startAsDaemon(() ->
      {
         Runtime runtime = Runtime.getRuntime();
         try
         {
            // -C include UDP
            // -t make nethogs print sequentially to console rather than the fullscreen rewriting display; so we can parse it
            // -a include all network interfaces
            process = runtime.exec("sudo nethogs -C -t -a -d 0.2");
            bufferedReader = new BufferedReader(new InputStreamReader(process.getInputStream()));

            nethogsReaderThread.start();

            int read;
            while ((read = bufferedReader.read()) > -1)
            {
               synchronized (syncObject)
               {
                  characterQueue.add(read);
               }
            }

            try
            {
               process.waitFor();
            }
            catch (InterruptedException e)
            {
               LogTools.error(e.getMessage());
               e.printStackTrace();
            }
         }
         catch (IOException e)
         {
            LogTools.error(e.getMessage());
            e.printStackTrace();
         }
      }, "Nethogs");
   }

   private void processNethogsOutput()
   {
      synchronized (syncObject)
      {
         if (!characterQueue.isEmpty())
         {
            StringBuilder output = new StringBuilder();
            while (!characterQueue.isEmpty())
            {
               output.append((char) ((int) characterQueue.poll()));
            }

            String outputThisTime = output.toString();

            if (!outputThisTime.isEmpty())
            {
               String[] lines = outputThisTime.split("\\R");
               for (String line : lines)
               {
                  String lineTrimmed = line.trim();
                  if (!lineTrimmed.isEmpty())
                  {
                     String[] pieces = lineTrimmed.split("\\t");
                     if (pieces[0].startsWith("Refreshing"))
                     {
                        kilobytesPerSecondSent = kilobytesPerSecondSentTemp;
                        kilobytesPerSecondReceived = kilobytesPerSecondReceivedTemp;
                        kilobytesPerSecondSentTemp = 0.0;
                        kilobytesPerSecondReceivedTemp = 0.0;
                     }
                     if (pieces.length == 3)
                     {
                        kilobytesPerSecondSentTemp += Double.parseDouble(pieces[1]);
                        kilobytesPerSecondReceivedTemp += Double.parseDouble(pieces[2]);
                     }
                  }
               }
            }
         }
      }
   }

   public void stop()
   {
      nethogsReaderThread.stop();

      if (process != null)
      {
         process.destroy();
      }
   }

   public double getKilobytesPerSecondSent()
   {
      return kilobytesPerSecondSent;
   }

   public double getKilobytesPerSecondReceived()
   {
      return kilobytesPerSecondReceived;
   }

   public static void main(String[] args)
   {
      NethogsNetworkMonitor nethogsNetworkMonitor = new NethogsNetworkMonitor();
      nethogsNetworkMonitor.start();

      PausablePeriodicThread thread = new PausablePeriodicThread("SendReceive", 0.2, false, () ->
      {
         LogTools.info("Sent: {}", nethogsNetworkMonitor.getKilobytesPerSecondSent());
         LogTools.info("Received: {}", nethogsNetworkMonitor.getKilobytesPerSecondReceived());
      });
      thread.start();
   }
}
