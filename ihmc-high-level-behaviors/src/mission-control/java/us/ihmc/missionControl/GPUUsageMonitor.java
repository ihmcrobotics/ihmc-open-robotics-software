package us.ihmc.missionControl;

import us.ihmc.log.LogTools;
import us.ihmc.tools.processManagement.ProcessTools;

public class GPUUsageMonitor
{
   private boolean hasNvidiaGPU = false;

   public GPUUsageMonitor()
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

   public double getGPUUsage()
   {
      double gpuUsage = Double.NaN;
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
               return Double.parseDouble(splitByWhitespace[2]);
            }
         }
      }
      return gpuUsage;
   }

   public boolean getHasNvidiaGPU()
   {
      return hasNvidiaGPU;
   }
}
