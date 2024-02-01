package us.ihmc.tools.io;

import us.ihmc.tools.io.resources.ResourceTools;

import java.io.InputStream;
import java.net.URL;
import java.nio.file.Path;
import java.nio.file.Paths;

/**
 * Used with {@link WorkspaceResourceDirectory} to represent a file in the
 * resources classpath.
 */
public class WorkspaceResourceFile extends WorkspaceFile
{
   private final Class<?> classForLoading;
   private final Path resourceFilePath;
   private final String pathAsStringForResourceLoading;

   public WorkspaceResourceFile(WorkspaceResourceDirectory resourceDirectory, String subsequentPathToFile)
   {
      super(resourceDirectory, subsequentPathToFile);

      classForLoading = resourceDirectory.getClassForLoading();
      resourceFilePath = Paths.get(resourceDirectory.getPathNecessaryForClasspathLoading()).resolve(subsequentPathToFile);
      pathAsStringForResourceLoading = ResourceTools.toResourceAccessStringWithCorrectSeparators(resourceFilePath);
   }

   public InputStream getClasspathResourceAsStream()
   {
      return classForLoading.getResourceAsStream(pathAsStringForResourceLoading);
   }

   /**
    * @return null if not found
    */
   public URL getClasspathResource()
   {
      return classForLoading.getResource(pathAsStringForResourceLoading);
   }

   public String getFileName()
   {
      return resourceFilePath.getFileName().toString();
   }

   public String getPathForResourceLoadingPathFiltered()
   {
      return pathAsStringForResourceLoading;
   }
}
