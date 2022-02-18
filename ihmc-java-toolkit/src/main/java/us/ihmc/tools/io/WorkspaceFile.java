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

   public WorkspaceFile(WorkspaceDirectory directory, String subsequentPathToFile)
   {
      String pathForResourceLoading = Paths.get(directory.getPathNecessaryForClasspathLoading()).resolve(subsequentPathToFile).toString();
      // Get rid of Windows \ slashes; they don't work with classloader
      String pathForResourceLoadingPathFiltered = pathForResourceLoading.replaceAll("\\\\", "/");
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

   public InputStream getClasspathResourceAsStream()
   {
      return getResourceAsStream.get();
   }

   public URL getClasspathResource()
   {
      return getResource.get();
   }

   public Path getFilePath()
   {
      return workspaceFile;
   }
}
