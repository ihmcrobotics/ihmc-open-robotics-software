package us.ihmc.missionControl;

import us.ihmc.tools.processManagement.ProcessTools;

import java.util.ArrayList;

public class LinuxResourceMonitor
{
   private float totalRAMGiB;
   private float usedRAMGiB;

   private final ArrayList<CPUCoreTracker> cpuCoreTrackers = new ArrayList<>();
   private final ArrayList<String> cpuLines = new ArrayList<>();

   public LinuxResourceMonitor()
   {
   }

   public void update()
   {
      calculateRAMUsage();
      calculateCPUUsage();
   }

   private void calculateRAMUsage()
   {
      String free = ProcessTools.execSimpleCommandSafe("free --mebi");
      String[] lines = free.split("\\R");
      String[] amongSpaces = lines[1].split("\\s+");
      totalRAMGiB = (float) (Float.parseFloat(amongSpaces[1]) / 1000.0);
      usedRAMGiB = (float) (Float.parseFloat(amongSpaces[2]) / 1000.0);
   }

   private void calculateCPUUsage()
   {
      String procStat = ProcessTools.execSimpleCommandSafe("cat /proc/stat");
      String[] lines = procStat.split("\\R");
      cpuLines.clear();
      for (String line : lines)
      {
         if (line.matches("^cpu\\d.*"))
         {
            cpuLines.add(line);
         }
         if (cpuCoreTrackers.size() < cpuLines.size())
         {
            cpuCoreTrackers.add(new CPUCoreTracker());
         }
      }

      for (int i = 0; i < cpuLines.size(); i++)
      {
         String cpuLine = cpuLines.get(i);
         String[] split = cpuLine.split("\\s+");
         // https://www.baeldung.com/linux/get-cpu-usage
         float timeInUserMode = Float.parseFloat(split[1]);
         float timeNiceUserMode = Float.parseFloat(split[2]);
         float timeInKernelCode = Float.parseFloat(split[3]);
         float idleTime = Float.parseFloat(split[4]);
         float ioWaitTime = Float.parseFloat(split[5]);
         float interruptTime = Float.parseFloat(split[6]);
         float softIRQTime = Float.parseFloat(split[7]);
         //         float stealTime = Float.parseFloat(split[8]);
         //         float guestTime = Float.parseFloat(split[9]);
         //         float guestNiceTime = Float.parseFloat(split[10]);

         cpuCoreTrackers.get(i).update(timeInUserMode, timeNiceUserMode, timeInKernelCode, idleTime, ioWaitTime, interruptTime, softIRQTime);
      }
   }

   public ArrayList<CPUCoreTracker> getCpuCoreTrackers()
   {
      return cpuCoreTrackers;
   }

   public float getTotalRAMGiB()
   {
      return totalRAMGiB;
   }

   public float getUsedRAMGiB()
   {
      return usedRAMGiB;
   }
}
