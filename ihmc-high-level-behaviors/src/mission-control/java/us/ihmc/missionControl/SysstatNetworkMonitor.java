package us.ihmc.missionControl;

import us.ihmc.log.LogTools;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

// https://github.com/sysstat/sysstat/wiki#2-questions-related-to-sar-sadc-and-sadf
public class SysstatNetworkMonitor extends ProcessOutputMonitor
{
   private final Map<String, Float> ifaceRxKbps = new ConcurrentHashMap<>();
   private final Map<String, Float> ifaceTxKbps = new ConcurrentHashMap<>();

   public SysstatNetworkMonitor()
   {
      super("sar", "-n", "DEV", "3");
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
