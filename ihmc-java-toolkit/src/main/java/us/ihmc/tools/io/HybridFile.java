package us.ihmc.tools.io;

import java.nio.file.Path;

/**
 * Used with {@link HybridDirectory} to represent a file.
 */
public class HybridFile
{
   protected Path externalFile;
   protected WorkspaceFile workspaceFile;
   protected HybridResourceMode mode = HybridResourceMode.WORKSPACE;

   protected HybridFile()
   {

   }

   public HybridFile(HybridDirectory directory, String subsequentPathToFile)
   {
      externalFile = directory.getExternalDirectory().resolve(subsequentPathToFile);
      workspaceFile = new WorkspaceFile(directory.getWorkspaceDirectoryInternal(), subsequentPathToFile);
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
      return mode == HybridResourceMode.WORKSPACE ? workspaceFile.getFilesystemFile() : externalFile;
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

   public Path getExternalFile()
   {
      return externalFile;
   }

   public Path getWorkspaceFile()
   {
      return workspaceFile.getFilesystemFile();
   }
}
