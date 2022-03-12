package us.ihmc.tools.io;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.nio.FileTools;

import java.io.BufferedInputStream;
import java.io.FileInputStream;
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
   private HybridResourceMode mode = HybridResourceMode.WORKSPACE;

   public HybridFile(HybridDirectory directory, String subsequentPathToFile)
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

      externalFile = directory.getExternalDirectory().resolve(subsequentPathToFile);
      if (directory.isWorkspaceWritingAvailable())
         workspaceFile = directory.getWorkspaceDirectory().resolve(subsequentPathToFile);
      else
         workspaceFile = null;
   }

   /**
    * i.e. Cannot write to resource directories inside JARs
    */
   public boolean isWorkspaceWritingAvailable()
   {
      return workspaceFile != null;
   }

   public void setMode(HybridResourceMode mode)
   {
      this.mode = mode;
   }

   public HybridResourceMode getMode()
   {
      return mode;
   }

   public Path getFileForWriting()
   {
      return mode == HybridResourceMode.WORKSPACE ? workspaceFile : externalFile;
   }

   public InputStream getInputStream()
   {
      return mode == HybridResourceMode.WORKSPACE ?
            getClasspathResourceAsStream() :
            ExceptionTools.handle(() -> new FileInputStream(externalFile.toFile()), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
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
