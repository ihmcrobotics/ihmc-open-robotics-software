package us.ihmc.commons.nio;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.apache.commons.lang3.StringUtils;

public class CommonPaths
{
   private static final String TEST_RESOURCES_FOLDER_NAME = "testResources";
   private static final String RESOURCES_FOLDER_NAME = "resources";
   
   public static Path deriveResourcesPath(Class<?> clazz)
   {
      List<String> pathNames = new ArrayList<String>();
      
      String[] packageNames = clazz.getPackage().getName().split("\\.");
      
      pathNames.addAll(Arrays.asList(packageNames));
      pathNames.add(StringUtils.uncapitalize(clazz.getSimpleName()));
      
      return Paths.get(RESOURCES_FOLDER_NAME, pathNames.toArray(new String[0]));
   }

   public static Path deriveTestResourcesPath(Class<?> clazz)
   {
      List<String> pathNames = new ArrayList<String>();
      
      String[] packageNames = clazz.getPackage().getName().split("\\.");
      
      pathNames.addAll(Arrays.asList(packageNames));
      pathNames.add(StringUtils.uncapitalize(clazz.getSimpleName()));
      
      return Paths.get(TEST_RESOURCES_FOLDER_NAME, pathNames.toArray(new String[0]));
   }
}
