package us.ihmc.missionControl;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.tools.thread.PausablePeriodicThread;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentLinkedQueue;

// https://github.com/sysstat/sysstat/wiki#2-questions-related-to-sar-sadc-and-sadf
public class SysstatNetworkMonitor
{
   private final ConcurrentLinkedQueue<Integer> characterQueue = new ConcurrentLinkedQueue<>();
   private final PausablePeriodicThread sarReaderThread;

   private Map<String, Float> ifaceRxKbps = new ConcurrentHashMap<>();
   private Map<String, Float> ifaceTxKbps = new ConcurrentHashMap<>();

   private static volatile boolean running = true;

   public SysstatNetworkMonitor()
   {
      sarReaderThread = new PausablePeriodicThread("sar-reader", 0.2, true, this::processSarOutput);
      sarReaderThread.start();
      start();
   }

   public Map<String, Float> getIfaceRxKbps()
   {
      return ifaceRxKbps;
   }

   public Map<String, Float> getIfaceTxKbps()
   {
      return ifaceTxKbps;
   }

   public void stop()
   {
      running = false;
   }

   public void start()
   {
      ThreadTools.startAsDaemon(() ->
      {
         ProcessBuilder processBuilder = new ProcessBuilder("sar", "-n", "DEV", "3");

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
      }, "sar-process");
   }

   private void processSarOutput()
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

         /*
          * Example
          * 0        1      2       3         4          5         6        7         8        9          10
          * 10:25:22 AM     IFACE   rxpck/s   txpck/s    rxkB/s    txkB/s   rxcmp/s   txcmp/s  rxmcst/s   %ifutil
          * 10:25:25 AM        lo      1.67      1.67      0.10      0.10      0.00      0.00      0.00      0.00
          */

         // Loop through each line
         for (int i = 0; i < lines.length; i++)
         {
            if (i < 2)
               continue; // Skip the table header

            String[] values = lines[i].split("\\s+");

            String time;
            String ampm;
            String iface;
            String rxpcks;
            String txpcks;
            String rxkbs;
            String txkbs;
            String rxcmps;
            String txcmps;
            String rxmcsts;
            String pctifutil;

            try
            {
               time = values[0].trim();
               ampm = values[1].trim();
               iface = values[2].trim();
               rxpcks = values[3].trim();
               txpcks = values[4].trim();
               rxkbs = values[5].trim();
               txkbs = values[6].trim();
               rxcmps = values[7].trim();
               txcmps = values[8].trim();
               rxmcsts = values[9].trim();
               pctifutil = values[10].trim();
            }
            catch (ArrayIndexOutOfBoundsException ignored)
            {
               LogTools.info("Unable to parse sar network stats table");
               return;
            }

            try
            {
               ifaceRxKbps.put(iface, Float.parseFloat(rxkbs));
               ifaceTxKbps.put(iface, Float.parseFloat(txkbs));
            }
            catch (NumberFormatException ignored)
            {
               LogTools.info("Unable to parse rx/tx from sar network stats table");
               return;
            }
         }
      }
   }

   public static void main(String[] args) throws IOException, InterruptedException
   {
      SysstatNetworkMonitor networkMonitor = new SysstatNetworkMonitor();
      networkMonitor.start();

      new PausablePeriodicThread("rx-tx-print", 1, false, () ->
      {
         networkMonitor.getIfaceRxKbps().forEach((iface, rxKbps) -> LogTools.info(iface + "rx (kbps) : " + rxKbps));
         networkMonitor.getIfaceTxKbps().forEach((iface, txKbps) -> LogTools.info(iface + " tx (kbps) : " + txKbps));
      }).start();
   }
}
