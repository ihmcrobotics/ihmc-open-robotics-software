package us.ihmc.tools.io;

import javax.annotation.Nullable;
import java.nio.file.Path;

/**
 * This is used with WorkspaceDirectory to work with source files in version control.
 */
public class WorkspaceFile
{
   /**
    * Should be kept absolute, otherwise will break Windows because it won't
    * have the drive letter i.e. "C:"
    *
    * This will be null in the case where filesystem access of this file
    * is not available, such as when this file refers to a resource on the classpath
    * and, in this process, that classpath resource is loaded from a JAR file.
    */
   @Nullable
   private final Path filesystemFile;

   public WorkspaceFile(WorkspaceDirectory directory, String subsequentPathToFile)
   {
      if (directory.isFileAccessAvailable())
      {
         filesystemFile = directory.getFilesystemDirectory().resolve(subsequentPathToFile).toAbsolutePath();
      }
      else
      {
         filesystemFile = null;
      }
   }

   /** If the directory is available for reading/writing using files.
    *  If not, we are running from a JAR without the resource extracted,
    *  or the working directory is wrong. */
   public boolean isFileAccessAvailable()
   {
      return filesystemFile != null;
   }

   public Path getFilesystemFile()
   {
      return filesystemFile;
   }
}
