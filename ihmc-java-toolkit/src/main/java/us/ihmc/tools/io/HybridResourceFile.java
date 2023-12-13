package us.ihmc.tools.io;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.log.LogTools;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.URL;
import java.nio.file.Files;
import java.util.function.Consumer;

/**
 * See {@link HybridFile}. This class extends that to provide additional assistance when
 * that file is a resource in the resources classpath.
 */
public class HybridResourceFile extends HybridFile
{
   private final WorkspaceResourceFile workspaceResourceFile;

   public HybridResourceFile(HybridResourceDirectory hybridResourceDirectory, String subsequentPathToResource)
   {
      externalFile = hybridResourceDirectory.getExternalDirectory().resolve(subsequentPathToResource);
      workspaceResourceFile = new WorkspaceResourceFile(hybridResourceDirectory.getWorkspaceResourceDirectoryInternal(), subsequentPathToResource);
      workspaceFile = workspaceResourceFile;
   }

   /**
    * Get this file as an input stream, if possible, and close it afterwards.
    *
    * @return if the input stream was consumed successfully
    */
   public boolean getInputStream(Consumer<InputStream> inputStreamGetter)
   {
      boolean success = false;
      try (InputStream inputStream = getInputStreamUnsafe())
      {
         if (inputStream != null)
         {
            inputStreamGetter.accept(inputStream);
            success = true;
         }
         else
         {
            // We return success so the caller can decide when to throw errors
            // because it may be normal for this to be null.
            LogTools.debug(1, "Input stream is null"); // Print caller info to help identify issues
         }
      }
      catch (IOException ioException)
      {
         DefaultExceptionHandler.MESSAGE_AND_STACKTRACE.handleException(ioException);
      }
      return success;
   }

   private InputStream getInputStreamUnsafe()
   {
      return mode == HybridResourceMode.WORKSPACE ?
            getClasspathResourceAsStream() :
            ExceptionTools.handle(() -> new FileInputStream(externalFile.toFile()), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
   }

   public boolean isInputStreamAvailable()
   {
      boolean isInputStreamAvailable;
      if (mode == HybridResourceMode.WORKSPACE)
      {
         isInputStreamAvailable = workspaceResourceFile.getClasspathResource() != null;
      }
      else
      {
         isInputStreamAvailable = Files.exists(externalFile);
      }
      return isInputStreamAvailable;
   }

   public String getLocationOfResourceForReading()
   {
      if (mode == HybridResourceMode.WORKSPACE)
      {
         return "Resource: " + getPathForResourceLoadingPathFiltered();
      }
      else
      {
         return "File: " + externalFile.toString();
      }
   }

   public InputStream getClasspathResourceAsStream()
   {
      return workspaceResourceFile.getClasspathResourceAsStream();
   }

   public URL getClasspathResource()
   {
      return workspaceResourceFile.getClasspathResource();
   }

   public String getPathForResourceLoadingPathFiltered()
   {
      return workspaceResourceFile.getPathForResourceLoadingPathFiltered();
   }
}
