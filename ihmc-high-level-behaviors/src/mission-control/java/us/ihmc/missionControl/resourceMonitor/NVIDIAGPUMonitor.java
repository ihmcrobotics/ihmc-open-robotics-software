package us.ihmc.missionControl.resourceMonitor;

import us.ihmc.log.LogTools;

// TODO: support multiple GPUs
public class NVIDIAGPUMonitor extends ResourceMonitor
{
   private String gpuModel;
   private int gpuUsage;
   private int memoryUsage;
   private int encoderUsage;
   private int decoderUsage;
   private int memoryTotal;
   private int memoryReserved;
   private int memoryUsed;
   private int memoryFree;
   private int temperature;

   public NVIDIAGPUMonitor()
   {
      super("nvidia-smi", "--query");
   }

   public String getGpuModel()
   {
      return gpuModel;
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

   public int getTemperature()
   {
      return temperature;
   }

   @Override
   public void parse(String[] lines)
   {
      for (int i = 0; i < lines.length; i++)
      {
         String line = lines[i].trim();

         if (line.startsWith("Product Name"))
         {
            gpuModel = line.substring(line.indexOf(":") + 1).trim();
         }

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

         if (line.equals("Temperature"))
         {
            try
            {
               String currentTemperatureLine = lines[i + 1].trim();

               int currentTemperature = Integer.parseInt(currentTemperatureLine.split("\\s+")[4]);

               this.temperature = currentTemperature;
            }
            catch (ArrayIndexOutOfBoundsException | NumberFormatException ignored)
            {
               LogTools.info("Unable to parse nvidia-smi Temperature");
            }
         }
      }
   }
}
