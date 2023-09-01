package us.ihmc.avatar.reachabilityMap;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.commons.nio.FileTools;

public interface ReachabilityMapFileWriter
{
   default void write(Class<?> classForFilePath, ReachabilityMapRobotInformation robotInformation, Voxel3DGrid reachabilityMap)
   {
      File outputStream = newFile(robotInformation.getRobotDefinition().getName(), classForFilePath, getFileExtension());
      if (outputStream == null)
         return;
      write(outputStream, robotInformation, reachabilityMap);
   }

   void write(File file, ReachabilityMapRobotInformation robotInformation, Voxel3DGrid reachabilityMap);

   String getFileExtension();

   static File newFile(String robotName, Class<?> classForFilePath, String fileExtension)
   {
      String fileName = ReachabilityMapFileWriter.prependDateToFileName(robotName) + fileExtension;
      Path filePath = deriveResourcesPath(classForFilePath);
      try
      {
         FileTools.ensureDirectoryExists(filePath);
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return null;
      }
      filePath = filePath.resolve(fileName);

      return filePath.toFile();
   }

   static Path deriveResourcesPath(Class<?> clazz)
   {
      List<String> pathNames = new ArrayList<String>();

      String[] packageNames = clazz.getPackage().getName().split("\\.");

      pathNames.addAll(Arrays.asList(packageNames));
      pathNames.add(StringUtils.uncapitalize(clazz.getSimpleName()));

      return Paths.get("src/main/resources", pathNames.toArray(new String[0]));
   }

   static String prependDateToFileName(String fileName)
   {
      DateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss_");
      Date date = new Date();
      String dateAsString = dateFormat.format(date);

      return dateAsString + fileName;
   }
}
