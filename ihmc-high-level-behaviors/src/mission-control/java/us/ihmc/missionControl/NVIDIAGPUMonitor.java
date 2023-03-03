package us.ihmc.missionControl;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.tools.processManagement.ProcessTools;

// TODO: support multiple GPUs
public class NVIDIAGPUMonitor
{
   private int gpuUsage;
   private int memoryUsage;
   private int encoderUsage;
   private int decoderUsage;
   private int memoryTotal;
   private int memoryReserved;
   private int memoryUsed;
   private int memoryFree;

   private boolean hasNvidiaGPU = false;

   public NVIDIAGPUMonitor()
   {
      String nvidiaSMI = ProcessTools.execSimpleCommand("nvidia-smi");
      if (nvidiaSMI != null && !nvidiaSMI.isEmpty())
      {
         hasNvidiaGPU = true;
      }
      LogTools.info("Has NVIDIA GPU: {}", hasNvidiaGPU);
   }

   public int getGpuUsage()
   {
      return gpuUsage;
   }

   public int getMemoryUsage()
   {
      return memoryUsage;
   }

   public int getEncoderUsage()
   {
      return encoderUsage;
   }

   public int getDecoderUsage()
   {
      return decoderUsage;
   }

   public int getMemoryTotal()
   {
      return memoryTotal;
   }

   public int getMemoryReserved()
   {
      return memoryReserved;
   }

   public int getMemoryUsed()
   {
      return memoryUsed;
   }

   public int getMemoryFree()
   {
      return memoryFree;
   }

   public void update()
   {
      if (!hasNvidiaGPU)
         return;

      String smiQuery = ProcessTools.execSimpleCommand("nvidia-smi --query --display=MEMORY,UTILIZATION");

      if (smiQuery == null)
      {
         hasNvidiaGPU = false;
         return;
      }

      String[] lines = smiQuery.split("\n");

      for (int i = 0; i < lines.length; i++)
      {
         String line = lines[i].trim();

         if (line.equals("FB Memory Usage"))
         {
            try
            {
               String totalLine = lines[i + 1].trim();
               String reservedLine = lines[i + 2].trim();
               String usedLine = lines[i + 3].trim();
               String freeLine = lines[i + 4].trim();

               int total = Integer.parseInt(totalLine.split("\\s+")[2]);
               int reserved = Integer.parseInt(reservedLine.split("\\s+")[2]);
               int used = Integer.parseInt(usedLine.split("\\s+")[2]);
               int free = Integer.parseInt(freeLine.split("\\s+")[2]);

               this.memoryTotal = total;
               this.memoryReserved = reserved;
               this.memoryUsed = used;
               this.memoryFree = free;
            }
            catch (ArrayIndexOutOfBoundsException | NumberFormatException ignored)
            {
               LogTools.info("Unable to parse nvidia-smi FB Memory Usage");
            }
         }

         if (line.equals("Utilization"))
         {
            try
            {
               String gpuLine = lines[i + 1].trim();
               String memoryLine = lines[i + 2].trim();
               String encoderLine = lines[i + 3].trim();
               String decoderLine = lines[i + 4].trim();

               int gpu = Integer.parseInt(gpuLine.split("\\s+")[2]);
               int memory = Integer.parseInt(memoryLine.split("\\s+")[2]);
               int encoder = Integer.parseInt(encoderLine.split("\\s+")[2]);
               int decoder = Integer.parseInt(decoderLine.split("\\s+")[2]);

               this.gpuUsage = gpu;
               this.memoryUsage = memory;
               this.encoderUsage = encoder;
               this.decoderUsage = decoder;
            }
            catch (ArrayIndexOutOfBoundsException | NumberFormatException ignored)
            {
               LogTools.info("Unable to parse nvidia-smi Utilization");
            }
         }
      }
   }

   public boolean getHasNvidiaGPU()
   {
      return hasNvidiaGPU;
   }

   public static void main(String[] args)
   {
      NVIDIAGPUMonitor nvidiagpuMonitor = new NVIDIAGPUMonitor();

      while (true)
      {
         nvidiagpuMonitor.update();
         ThreadTools.sleep(1000);
      }
   }
}
