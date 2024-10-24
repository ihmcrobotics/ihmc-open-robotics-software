package us.ihmc.behaviors.activeMapping;

import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.nio.WriteOption;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.perception.logging.PerceptionDataLogger;
import us.ihmc.perception.logging.PerceptionLoggerConstants;
import us.ihmc.perception.tools.PerceptionLoggingTools;
import us.ihmc.tools.IHMCCommonPaths;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.time.Instant;
import java.util.Date;
import java.util.HashMap;

public class ContinuousHikingLogger
{
   private static final boolean DEBUG = false;
   private File file;
   private final HashMap<String, Float> statistics = new HashMap<>();

   private final PerceptionDataLogger perceptionDataLogger = new PerceptionDataLogger();

   StringBuilder additionalString = new StringBuilder();

   public ContinuousHikingLogger()
   {
      SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
      String logFileName = dateFormat.format(new Date()) + "_" + "ContinuousPlannerLog.txt";
      FileTools.ensureDirectoryExists(IHMCCommonPaths.CONTINUOUS_HIKING_DIRECTORY, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      String filePath = IHMCCommonPaths.CONTINUOUS_HIKING_DIRECTORY.resolve(logFileName).toString();

      try
      {
         if(!Files.exists(IHMCCommonPaths.CONTINUOUS_HIKING_DIRECTORY))
         {
            Files.createDirectory(IHMCCommonPaths.CONTINUOUS_HIKING_DIRECTORY);
         }
         if (!Files.exists(IHMCCommonPaths.TERRAIN_MAP_DIRECTORY.resolve(logFileName)))
         {
            Files.createFile(Paths.get(filePath));
            file = new File(filePath);
         }
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public void logToFile(boolean logToFile, boolean printToConsole)
   {
      if (logToFile || printToConsole)
      {
         if (printToConsole)
            System.out.println(this);

         if (logToFile)
         {
            if (DEBUG)
            LogTools.info("Logging Continuous Walking Statistics: {}", file.getAbsoluteFile().toPath());
            FileTools.write(file.getAbsoluteFile().toPath(), toString().getBytes(), WriteOption.APPEND, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
         }

         additionalString.setLength(0);
      }
   }

   public String toString()
   {
      StringBuilder builder = new StringBuilder();
      builder.append("ContinuousPlannerStatistics: [");

      for (String key : statistics.keySet())
      {
         builder.append(key).append(":").append(statistics.get(key)).append(", ");
      }

      builder.append("]\n");
      if (DEBUG)
         LogTools.warn("Additional String: {}", additionalString.toString());
      builder.append(additionalString.toString()).append("\n");

      return builder.toString();
   }

   public void appendString(boolean printToConsole ,String string)
   {
      if (DEBUG)
         LogTools.warn("Additional String: {}", string);
      additionalString.append(String.format("[%s]: (", new SimpleDateFormat("HH:mm:ss.SSS").format(new Date())));
      additionalString.append(string);
      additionalString.append(")\n");
   }
}
