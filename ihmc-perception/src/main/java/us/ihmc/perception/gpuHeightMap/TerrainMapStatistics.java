package us.ihmc.perception.gpuHeightMap;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.nio.WriteOption;
import us.ihmc.tools.IHMCCommonPaths;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.HashMap;

public class TerrainMapStatistics
{
   private File file;
   private final HashMap<String, Float> statistics = new HashMap<>();

   // These all need to be snake case in order for everything to work
   private static final String TOTAL_TIME = "total_time";
   private static final String DEPTH_UPLOAD_TIME = "depth_upload_time";
   private static final String TERRAIN_MAP_DOWNLOAD_TIME = "terrain_map_download_time";
   private static final String EXTRACTION_TIME = "extraction_time";
   private static final String CPU_PROCESSING_TIME = "cpu_processing_time";
   private static final String GPU_PROCESSING_TIME = "gpu_processing_time";

   private boolean printToConsole = false;

   public TerrainMapStatistics()
   {
      SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
      String logFileName = dateFormat.format(new Date()) + "_" + "TerrainMapStatistics.txt";
      FileTools.ensureDirectoryExists(IHMCCommonPaths.TERRAIN_MAP_DIRECTORY, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      String filePath = IHMCCommonPaths.TERRAIN_MAP_DIRECTORY.resolve(logFileName).toString();

      try
      {
         if (!Files.exists(IHMCCommonPaths.LOGS_DIRECTORY))
         {
            Files.createDirectory(IHMCCommonPaths.LOGS_DIRECTORY);
         }
         if(!Files.exists(IHMCCommonPaths.TERRAIN_MAP_DIRECTORY))
         {
            Files.createDirectory(IHMCCommonPaths.TERRAIN_MAP_DIRECTORY);
         }
         if (!Files.exists(IHMCCommonPaths.TERRAIN_MAP_DIRECTORY.resolve(logFileName)))
         {
            file = new File(filePath);
         }
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      statistics.put(TOTAL_TIME, 0.0f);
      statistics.put(DEPTH_UPLOAD_TIME, 0.0f);
      statistics.put(TERRAIN_MAP_DOWNLOAD_TIME, 0.0f);
      statistics.put(EXTRACTION_TIME, 0.0f);
      statistics.put(CPU_PROCESSING_TIME, 0.0f);
      statistics.put(GPU_PROCESSING_TIME, 0.0f);
   }

   public void startTotalTime()
   {
      statistics.put(TOTAL_TIME, (System.nanoTime() * 1e-6f));
   }

   public void endTotalTime()
   {
      statistics.put(TOTAL_TIME, ((System.nanoTime() * 1e-6f) - statistics.get(TOTAL_TIME)));
   }

   public void startDepthUploadTime()
   {
      statistics.put(DEPTH_UPLOAD_TIME, (System.nanoTime() * 1e-6f));
   }

   public void endDepthUploadTime()
   {
      statistics.put(DEPTH_UPLOAD_TIME, ((System.nanoTime() * 1e-6f) - statistics.get(DEPTH_UPLOAD_TIME)));
   }

   public void startTerrainMapDownloadTime()
   {
      statistics.put(TERRAIN_MAP_DOWNLOAD_TIME, (System.nanoTime() * 1e-6f));
   }

   public void endTerrainMapDownloadTime()
   {
      statistics.put(TERRAIN_MAP_DOWNLOAD_TIME, ((System.nanoTime() * 1e-6f) - statistics.get(TERRAIN_MAP_DOWNLOAD_TIME)));
   }

   public void startExtractionTime()
   {
      statistics.put(EXTRACTION_TIME, (System.nanoTime() * 1e-6f));
   }

   public void endExtractionTime()
   {
      statistics.put(EXTRACTION_TIME, ((System.nanoTime() * 1e-6f) - statistics.get(EXTRACTION_TIME)));
   }

   public void startCPUProcessingTime()
   {
      statistics.put(CPU_PROCESSING_TIME, (System.nanoTime() * 1e-6f));
   }

   public void endCPUProcessingTime()
   {
      statistics.put(CPU_PROCESSING_TIME, ((System.nanoTime() * 1e-6f) - statistics.get(CPU_PROCESSING_TIME)));
   }

   public void startGPUProcessingTime()
   {
      statistics.put(GPU_PROCESSING_TIME, (System.nanoTime() * 1e-6f));
   }

   public void endGPUProcessingTime()
   {
      statistics.put(GPU_PROCESSING_TIME, ((System.nanoTime() * 1e-6f) - statistics.get(GPU_PROCESSING_TIME)));
   }

   public void logToFile(boolean logToFile)
   {
      if (logToFile || printToConsole)
      {
         if (printToConsole)
            System.out.println(this);

         if (logToFile)
         {
//            LogTools.info("Logging Terrain Map Statistics: {}", file.getAbsoluteFile().toPath());
            FileTools.write(file.getAbsoluteFile().toPath(), toString().getBytes(), WriteOption.APPEND, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
         }
      }
   }

   public String toString()
   {
      StringBuilder builder = new StringBuilder();
      builder.append("TerrainMapStatistics: [");
      for (String key : statistics.keySet())
      {
         builder.append(key).append(":").append(String.format("%.3f", statistics.get(key))).append(", ");
      }
      builder.append("]\n");
      return builder.toString();
   }

   public void setPrintToConsole(boolean printToConsole)
   {
      this.printToConsole = printToConsole;
   }
}
