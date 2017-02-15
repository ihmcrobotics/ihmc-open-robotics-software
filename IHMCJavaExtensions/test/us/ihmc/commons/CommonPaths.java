package us.ihmc.commons;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class CommonPaths
{
   private static final String TEST_RESOURCES_FOLDER_NAME = "testResources";
   
   public static Path deriveTestResourcesPath(Class<?> clazz)
   {
      List<String> pathNames = new ArrayList<String>();
      
      String[] packageNames = clazz.getPackage().getName().split("\\.");
      
      pathNames.addAll(Arrays.asList(packageNames));
      pathNames.add(Character.toLowerCase(clazz.getSimpleName().charAt(0)) + clazz.getSimpleName().substring(1));
      
      return Paths.get(TEST_RESOURCES_FOLDER_NAME, pathNames.toArray(new String[0]));
   }
}
