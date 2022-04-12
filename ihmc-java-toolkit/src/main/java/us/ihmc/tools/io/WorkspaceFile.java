package us.ihmc.tools.io;

import java.io.InputStream;
import java.net.URL;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.function.Supplier;

public class WorkspaceFile
{
   private final Supplier<InputStream> getResourceAsStream;
   private final Supplier<URL> getResource;
   private final Path workspaceFile;
   private final String pathForResourceLoadingPathFiltered;

   public WorkspaceFile(WorkspaceDirectory directory, String subsequentPathToFile)
   {
      String pathForResourceLoading = Paths.get(directory.getPathNecessaryForClasspathLoading()).resolve(subsequentPathToFile).toString();
      // Get rid of Windows \ slashes; they don't work with classloader
      pathForResourceLoadingPathFiltered = pathForResourceLoading.replaceAll("\\\\", "/");
      if (directory.getClassForLoading() == null) // TODO: This is broken
      {
         getResourceAsStream = () -> ClassLoader.getSystemResourceAsStream(pathForResourceLoadingPathFiltered);
         getResource = () -> ClassLoader.getSystemResource(pathForResourceLoadingPathFiltered);
      }
      else
      {
         getResourceAsStream = () -> directory.getClassForLoading().getResourceAsStream(pathForResourceLoadingPathFiltered);
         getResource = () -> directory.getClassForLoading().getResource(pathForResourceLoadingPathFiltered);
      }

      workspaceFile = directory.getDirectoryPath().resolve(subsequentPathToFile);
   }

   /** If the directory is available for reading/writing using files.
    *  If not, we are running from a JAR without the resource extracted,
    *  or the working directory is wrong. */
   public boolean isFileAccessAvailable()
   {
      return workspaceFile != null;
   }

   public InputStream getClasspathResourceAsStream()
   {
      return getResourceAsStream.get();
   }

   public URL getClasspathResource()
   {
      return getResource.get();
   }

   public String getResourceName()
   {
      return Paths.get(getResource.get().getPath()).getFileName().toString();
   }

   public Path getFilePath()
   {
      return workspaceFile;
   }

   public String getPathForResourceLoadingPathFiltered()
   {
      return pathForResourceLoadingPathFiltered;
   }
}
