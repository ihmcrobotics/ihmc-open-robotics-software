package us.ihmc.tools.io;

import java.io.InputStream;
import java.net.URL;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.function.Supplier;

public class HybridFile
{
   private final Supplier<InputStream> getResourceAsStream;
   private final Supplier<URL> getResource;
   private final Path externalFile;
   private final Path workspaceFile;

   public HybridFile(HybridDirectory directory, String subsequentPathToFile)
   {
      String pathForResourceLoading = Paths.get(directory.getPathNecessaryForClasspathLoading()).resolve(subsequentPathToFile).toString();
      if (directory.getClassForLoading() == null) // TODO: This is broken
      {
         getResourceAsStream = () -> ClassLoader.getSystemResourceAsStream(pathForResourceLoading);
         getResource = () -> ClassLoader.getSystemResource(pathForResourceLoading);
      }
      else
      {
         getResourceAsStream = () -> directory.getClassForLoading().getResourceAsStream(pathForResourceLoading);
         getResource = () -> directory.getClassForLoading().getResource(pathForResourceLoading);
      }

      externalFile = directory.getExternalDirectory().resolve(subsequentPathToFile);
      workspaceFile = directory.getWorkspaceDirectory().resolve(subsequentPathToFile);
   }

   public InputStream getClasspathResourceAsStream()
   {
      return getResourceAsStream.get();
   }

   public URL getClasspathResource()
   {
      return getResource.get();
   }

   public Path getExternalFile()
   {
      return externalFile;
   }

   public Path getWorkspaceFile()
   {
      return workspaceFile;
   }
}
