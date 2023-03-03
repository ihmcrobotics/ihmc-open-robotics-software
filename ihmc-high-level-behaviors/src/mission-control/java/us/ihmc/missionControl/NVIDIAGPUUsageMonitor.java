package us.ihmc.missionControl;

import us.ihmc.log.LogTools;
import us.ihmc.tools.processManagement.ProcessTools;

// TODO: support multiple GPUs
public class NVIDIAGPUUsageMonitor
{
   private boolean hasNvidiaGPU = false;

   public NVIDIAGPUUsageMonitor()
   {
      String glxinfo = ProcessTools.execSimpleCommand("glxinfo");
      if (glxinfo != null)
      {
         String[] lines = glxinfo.split("\\R");
         for (String line : lines)
         {
            if (line.startsWith("OpenGL vendor string") && line.contains("NVIDIA"))
            {
               hasNvidiaGPU = true;
            }
         }
      }
      String nvidiaSMI = ProcessTools.execSimpleCommand("nvidia-smi");
      if (nvidiaSMI != null && !nvidiaSMI.isEmpty())
      {
         hasNvidiaGPU = true;
      }
      LogTools.info("Has NVIDIA GPU: {}", hasNvidiaGPU);
   }

   public float getGPUUsage()
   {
      float gpuUsage = Float.NaN;
      if (hasNvidiaGPU)
      {
         String smiQuery = ProcessTools.execSimpleCommand("nvidia-smi --query --display=MEMORY,UTILIZATION");
         String[] lines = smiQuery.split("\\R");
         boolean inUtilizationSection = false;
         for (String line : lines)
         {
            String trimmed = line.trim();
            if (trimmed.startsWith("Utilization"))
            {
               inUtilizationSection = true;
            }
            if (inUtilizationSection && trimmed.startsWith("Gpu"))
            {
               String[] splitByWhitespace = trimmed.split("\\s+");
               return Float.parseFloat(splitByWhitespace[2]);
            }
         }
      }
      return gpuUsage;
   }

   public int getGPUUsedMemoryMB()
   {
      int usedMemory = 0;
      if (hasNvidiaGPU)
      {
         String smiQuery = ProcessTools.execSimpleCommand("nvidia-smi --query --display=MEMORY,UTILIZATION");
         String[] lines = smiQuery.split("\\R");
         boolean inMemorySection = false;
         for (String line : lines)
         {
            String trimmed = line.trim();
            if (trimmed.startsWith("FB Memory Usage"))
            {
               inMemorySection = true;
            }
            if (inMemorySection && trimmed.startsWith("Used"))
            {
               String[] splitByWhitespace = trimmed.split("\\s+");
               return Integer.parseInt(splitByWhitespace[2]);
            }
         }
      }
      return usedMemory;
   }

   public int getGPUTotalMemoryMB()
   {
      int totalMemory = 0;
      if (hasNvidiaGPU)
      {
         String smiQuery = ProcessTools.execSimpleCommand("nvidia-smi --query --display=MEMORY,UTILIZATION");
         String[] lines = smiQuery.split("\\R");
         boolean inMemorySection = false;
         for (String line : lines)
         {
            String trimmed = line.trim();
            if (trimmed.startsWith("FB Memory Usage"))
            {
               inMemorySection = true;
            }
            if (inMemorySection && trimmed.startsWith("Total"))
            {
               String[] splitByWhitespace = trimmed.split("\\s+");
               return Integer.parseInt(splitByWhitespace[2]);
            }
         }
      }
      return totalMemory;
   }

   public boolean getHasNvidiaGPU()
   {
      return hasNvidiaGPU;
   }
}
