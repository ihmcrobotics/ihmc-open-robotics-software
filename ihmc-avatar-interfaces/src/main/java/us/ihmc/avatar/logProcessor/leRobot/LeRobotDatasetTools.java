package us.ihmc.avatar.logProcessor.leRobot;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.List;

/**
 * Assorted tools.
 * <p>
 * Part of the {@link LeRobotDataset} generation system from IHMC logs.
 */
public class LeRobotDatasetTools
{
   public static List<Path> findLeRobotDatasetSubdirectories(Path startDirectory)
   {
      try (var paths = Files.walk(startDirectory))
      {
         return paths.filter(Files::isDirectory).filter(directory -> Files.exists(directory.resolve("meta/info.json"))).toList();
      }
      catch (IOException e)
      {
         throw new RuntimeException("Error searching for dataset directories with meta/info.json", e);
      }
   }

   public static void appendLine(Path path, String line)
   {
      ExceptionTools.handle(() -> Files.writeString(path, "%s%n".formatted(line.trim()), StandardOpenOption.APPEND), DefaultExceptionHandler.PRINT_MESSAGE);
   }
}
