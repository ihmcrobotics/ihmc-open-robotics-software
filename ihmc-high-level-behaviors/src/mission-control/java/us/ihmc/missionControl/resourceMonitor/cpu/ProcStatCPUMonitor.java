package us.ihmc.missionControl.resourceMonitor.cpu;

import us.ihmc.log.LogTools;
import us.ihmc.missionControl.resourceMonitor.ResourceMonitor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

// Reference https://man7.org/linux/man-pages/man5/proc.5.html
public class ProcStatCPUMonitor extends ResourceMonitor
{
   private Map<Integer, CPUCoreTracker> cpuCoreTrackers = new HashMap<>();

   public ProcStatCPUMonitor()
   {
      super("cat", "/proc/stat");
   }

   public Map<Integer, CPUCoreTracker> getCpuCoreTrackers()
   {
      return cpuCoreTrackers;
   }

   @Override
   public void parse(String[] lines)
   {
      /*
         Example output
         cpu  5701652 441923 881355 117020539 121591 194794 131183 0 0 0
         cpu0 738598 58992 107865 14588400 13274 24414 9334 0 0 0
         cpu1 628237 56258 108314 14714891 24483 20654 8828 0 0 0
         cpu2 566487 58957 104464 14785848 19378 21226 10534 0 0 0
         cpu3 504021 45120 115008 14856340 16745 25432 5471 0 0 0
         cpu4 952494 45528 124905 14297560 12187 43707 81127 0 0 0
         cpu5 969349 53511 115251 14385898 11129 21938 6013 0 0 0
         cpu6 708309 56126 104128 14661379 12919 19002 5257 0 0 0
         cpu7 634153 67426 101418 14730218 11473 18416 4616 0 0 0
         intr 249067767 23 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 31 2857 8 0 0 0 0 35 0 0 0 0 0 0 2304077 2913831 40096313 89129 6575 2 25 14978 2916442 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
         ctxt 534273269
         btime 1678830371
         processes 190623
         procs_running 1
         procs_blocked 0
         softirq 148359287 2913679 8700449 359328 43297585 2257347 0 572150 58494583 17192 31746974
       */
      List<String> cpuLines = new ArrayList<>();
      for (String line : lines)
         if (line.matches("(cpu)(\\d).*"))
            cpuLines.add(line);

      for (int i = 0; i < cpuLines.size(); i++)
      {
         String line = cpuLines.get(i);

         String cpuName;
         int cpu;
         int user;
         int nice;
         int system;
         int idle;
         int iowait;
         int irq;
         int softirq;
         int steal;
         int guest;
         int guest_nice;

         String[] split = line.split(" ");

         try
         {
            cpuName = split[0];
            cpu = Integer.parseInt(cpuName.replace("cpu", ""));
            user = Integer.parseInt(split[1]);
            nice = Integer.parseInt(split[2]);
            system = Integer.parseInt(split[3]);
            idle = Integer.parseInt(split[4]);
            iowait = Integer.parseInt(split[5]);
            irq = Integer.parseInt(split[6]);
            softirq = Integer.parseInt(split[7]);
            steal = Integer.parseInt(split[8]);
            guest = Integer.parseInt(split[9]);
            guest_nice = Integer.parseInt(split[10]);
         }
         catch (NumberFormatException e)
         {
            LogTools.info("Unable to parse /proc/stat");
            continue;
         }

         if (!cpuCoreTrackers.containsKey(cpu))
            cpuCoreTrackers.put(cpu, new CPUCoreTracker());

         cpuCoreTrackers.get(cpu).update(user, nice, system, idle, iowait, irq, softirq);
      }
   }
}
