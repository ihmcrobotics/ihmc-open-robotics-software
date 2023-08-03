package us.ihmc.missionControl.resourceMonitor;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

// https://github.com/sysstat/sysstat/wiki#2-questions-related-to-sar-sadc-and-sadf
public class SysstatNetworkMonitor extends ResourceMonitor
{
   private final Map<String, Float> ifaceRxKbps = new ConcurrentHashMap<>();
   private final Map<String, Float> ifaceTxKbps = new ConcurrentHashMap<>();

   public SysstatNetworkMonitor()
   {
      super(2.0, "timeout", "2.1s", "sar", "-n", "DEV", "1");
   }

   public Map<String, Float> getIfaceRxKbps()
   {
      return ifaceRxKbps;
   }

   public Map<String, Float> getIfaceTxKbps()
   {
      return ifaceTxKbps;
   }

   @Override
   public void parse(String[] lines)
   {
      /*
       * Example
       * 0        1      2       3         4          5         6        7         8        9          10
       * 10:25:22 AM     IFACE   rxpck/s   txpck/s    rxkB/s    txkB/s   rxcmp/s   txcmp/s  rxmcst/s   %ifutil
       * 10:25:25 AM        lo      1.67      1.67      0.10      0.10      0.00      0.00      0.00      0.00
       */

      // Loop through each line
      for (int i = 0; i < lines.length; i++)
      {
         String line = lines[i];

         if (!line.matches("\\d+:\\d+:\\d.*"))
            continue; // Ignore any line that doesn't begin with a time
         if (line.contains("IFACE"))
            continue; // Ignore the table header

         String[] values = line.split("\\s+");

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

   public static void main(String[] args)
   {
      SysstatNetworkMonitor monitor = new SysstatNetworkMonitor();
      monitor.start();

      while (true)
      {
         LogTools.info("rx");
         for (Map.Entry<String, Float> entry : monitor.getIfaceRxKbps().entrySet())
         {
            LogTools.info("\tiface " + entry.getKey() + " " + entry.getValue());
         }

         LogTools.info("tx");
         for (Map.Entry<String, Float> entry : monitor.getIfaceTxKbps().entrySet())
         {
            LogTools.info("\tiface " + entry.getKey() + " " + entry.getValue());
         }

         ThreadTools.sleep(1000);
      }
   }
}
