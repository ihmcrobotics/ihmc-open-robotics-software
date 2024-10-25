package us.ihmc.behaviors.activeMapping;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.nio.WriteOption;
import us.ihmc.log.LogTools;
import us.ihmc.tools.IHMCCommonPaths;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.Comparator;
import java.util.Date;
import java.util.SortedSet;
import java.util.TreeSet;
import java.util.stream.Stream;

public class ContinuousHikingLogger
{
   private static final int NUMBER_OF_LOGS_TO_KEEP = 100;
   private static final boolean DEBUG = false;
   private File file;

   private static final String CONTINUOUS_HIKING_FILE_SUFFIX = "ContinuousHikingLog.txt";
   private static final SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
   private static final String logFileName = dateFormat.format(new Date()) + "_" + CONTINUOUS_HIKING_FILE_SUFFIX;
   private static final String filePath = IHMCCommonPaths.CONTINUOUS_HIKING_DIRECTORY.resolve(logFileName).toString();

   StringBuilder additionalString = new StringBuilder();

   public ContinuousHikingLogger()
   {
      FileTools.ensureDirectoryExists(IHMCCommonPaths.CONTINUOUS_HIKING_DIRECTORY, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      deleteOldLogs();

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
            FileTools.write(file.getAbsoluteFile().toPath(), toString().getBytes(), WriteOption.APPEND, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
         }

         additionalString.setLength(0);
      }
   }

   public String toString()
   {
      StringBuilder builder = new StringBuilder();
      builder.append("\n");

      builder.append("]\n");
      if (DEBUG)
         LogTools.warn("Additional String: {}", additionalString.toString());
      builder.append(additionalString.toString()).append("\n");

      return builder.toString();
   }

   public void appendString(String string)
   {
      additionalString.append(String.format("[%s]: (", new SimpleDateFormat("HH:mm:ss.SSS").format(new Date())));
      additionalString.append(string);
      additionalString.append(")\n");
   }

   /** Keeps around the recommended number of logs. */
   public static void deleteOldLogs()
   {
      deleteOldLogs(NUMBER_OF_LOGS_TO_KEEP, IHMCCommonPaths.CONTINUOUS_HIKING_DIRECTORY.toString());
   }

   /**
    * It's recommended to leave quite a few logs around, otherwise, we diminish the usefulness of the logging.
    * This method expects the folder to exist or it will throw an exception
    */
   public static void deleteOldLogs(int numberOfLogsToKeep, String directory)
   {
      SortedSet<Path> sortedLogFolderPaths = new TreeSet<>(Comparator.comparing(path1 -> path1.getFileName().toString()));

      try (Stream<Path> paths = Files.walk(Path.of(directory)))
      {
         paths.filter(Files::isRegularFile).forEach(dir ->
                                                  {
                                                     if (dir.getFileName().toString().endsWith(ContinuousHikingLogger.CONTINUOUS_HIKING_FILE_SUFFIX))
                                                     {
                                                        sortedLogFolderPaths.add(dir);
                                                     }
                                                  });
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      while (sortedLogFolderPaths.size() > numberOfLogsToKeep)
      {
         Path earliestLogDirectory = sortedLogFolderPaths.first();
         LogTools.warn("Deleting old log {}", earliestLogDirectory);
         FileTools.deleteQuietly(earliestLogDirectory);
         sortedLogFolderPaths.remove(earliestLogDirectory);
      }
   }

}
