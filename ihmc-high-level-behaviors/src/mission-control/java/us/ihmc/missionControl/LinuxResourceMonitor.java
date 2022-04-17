package us.ihmc.missionControl;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;

public class LinuxResourceMonitor
{
   private double totalRAMGiB;
   private double usedRAMGiB;

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
      String free = execSimpleCommand("free --mebi");
      String[] lines = free.split("\\R");
      String[] amongSpaces = lines[1].split("\\s+");
      totalRAMGiB = Double.parseDouble(amongSpaces[1]) / 1000.0;
      usedRAMGiB = Double.parseDouble(amongSpaces[2]) / 1000.0;
   }

   private void calculateCPUUsage()
   {
      String procStat = execSimpleCommand("cat /proc/stat");
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
         double timeInUserMode = Double.parseDouble(split[1]);
         double timeNiceUserMode = Double.parseDouble(split[2]);
         double timeInKernelCode = Double.parseDouble(split[3]);
         double idleTime = Double.parseDouble(split[4]);
         double ioWaitTime = Double.parseDouble(split[5]);
         double interruptTime = Double.parseDouble(split[6]);
         double softIRQTime = Double.parseDouble(split[7]);
         //         double stealTime = Double.parseDouble(split[8]);
         //         double guestTime = Double.parseDouble(split[9]);
         //         double guestNiceTime = Double.parseDouble(split[10]);

         cpuCoreTrackers.get(i).update(timeInUserMode, timeNiceUserMode, timeInKernelCode, idleTime, ioWaitTime, interruptTime, softIRQTime);
      }
   }

   public static String execSimpleCommand(String command)
   {
      Runtime runtime = Runtime.getRuntime();
      try
      {
         Process process = runtime.exec(command);
         BufferedReader bufferedReader = new BufferedReader(new InputStreamReader(process.getInputStream()));

         StringBuilder output = new StringBuilder();
         int read;
         while ((read = bufferedReader.read()) > -1)
         {
            output.append((char) read);
         }

         process.waitFor();

         return output.toString();
      }
      catch (IOException |InterruptedException e)
      {
         e.printStackTrace();
      }
      return null;
   }

   public ArrayList<CPUCoreTracker> getCpuCoreTrackers()
   {
      return cpuCoreTrackers;
   }

   public double getTotalRAMGiB()
   {
      return totalRAMGiB;
   }

   public double getUsedRAMGiB()
   {
      return usedRAMGiB;
   }
}
