package us.ihmc.tools.io;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;

import java.io.FileInputStream;
import java.io.InputStream;
import java.net.URL;
import java.nio.file.Files;
import java.nio.file.Path;

public class HybridFile
{
   private final WorkspaceFile workspaceFile;
   private final Path externalFile;
   private HybridResourceMode mode = HybridResourceMode.WORKSPACE;

   public HybridFile(HybridDirectory directory, String subsequentPathToFile)
   {
      workspaceFile = new WorkspaceFile(directory.getInternalWorkspaceDirectory(), subsequentPathToFile);
      externalFile = directory.getExternalDirectory().resolve(subsequentPathToFile);
   }

   /** If the directory is available for reading/writing using files.
    *  If not, we are running from a JAR without the resource extracted,
    *  or the working directory is wrong. */
   public boolean isWorkspaceFileAccessAvailable()
   {
      return workspaceFile.isFileAccessAvailable();
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
      return mode == HybridResourceMode.WORKSPACE ? workspaceFile.getFilePath() : externalFile;
   }

   public InputStream getInputStream()
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
         isInputStreamAvailable = workspaceFile.getClasspathResource() != null;
      }
      else
      {
         isInputStreamAvailable = Files.exists(externalFile);
      }
      return isInputStreamAvailable;
   }

   public boolean isWritingAvailable()
   {
      boolean isWritingAvailable;
      if (mode == HybridResourceMode.WORKSPACE)
      {
         isWritingAvailable = isWorkspaceFileAccessAvailable();
      }
      else
      {
         isWritingAvailable = true; // Can always write externally
      }
      return isWritingAvailable;
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
      return workspaceFile.getClasspathResourceAsStream();
   }

   public URL getClasspathResource()
   {
      return workspaceFile.getClasspathResource();
   }

   public Path getExternalFile()
   {
      return externalFile;
   }

   public Path getWorkspaceFile()
   {
      return workspaceFile.getFilePath();
   }

   public String getPathForResourceLoadingPathFiltered()
   {
      return workspaceFile.getPathForResourceLoadingPathFiltered();
   }
}
