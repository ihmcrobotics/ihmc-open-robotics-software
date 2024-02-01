package us.ihmc.tools.io;

import us.ihmc.commons.nio.PathTools;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.resources.ResourceTools;

import javax.annotation.Nullable;
import java.nio.file.Path;

/**
 * Represents a directory in your source version controlled workspace.
 *
 * This class is designed for development, when building and running code from source in IntelliJ
 * or Eclipse, to provide the ability for applications to save files to version controlled
 * directories.
 *
 * The use of this class should therefore be limited to the above scenario. If saving files to
 * version controlled directories at runtime is not desired, the use another way.
 * See {@link us.ihmc.commons.nio.FileTools} and {@link ResourceTools}.
 *
 * There are a few different ways to use this class. Because the working directory on developers
 * machines is not consistent, we have to make some assumptions and use a few different strategies
 * in order to make this work.
 *
 * This class may fail to find the correct directory in special circumstances, in which you must
 * check isFileAccessAvailable() and if that's false, make sure to not try and use this class
 * from that point on. Please try to not crash the entire application with a null pointer in
 * that case, but instead just disable the save functionality for the duration of the run.
 */
public class WorkspaceDirectory
{
   /**
    * Should be kept absolute, otherwise will break Windows because it won't
    * have the drive letter i.e. "C:"
    *
    * This will be null in the case where filesystem access of this directory
    * is not available, such as when this directory refers to one on the classpath
    * and, in this process, that part of the classpath is loaded from a JAR file.
    */
   @Nullable
   protected Path filesystemDirectory;

   /**
    * This is used for the extending classes, which need to do some calculation before
    * setting filesystemDirectory, which is also why the filesystemDirectory can't be
    * final.
    */
   protected WorkspaceDirectory()
   {

   }

   /**
    * This way of construction looks for a directory in your working directory
    * or an immediate descendant of it.
    *
    * @param directoryNameToAssumePresent
    */
   public WorkspaceDirectory(String directoryNameToAssumePresent)
   {
      Path path = PathTools.findDirectoryInline(directoryNameToAssumePresent);
      filesystemDirectory = path == null ? null : path.toAbsolutePath();
   }

   /**
    * This way of construction looks for a directory in your working directory
    * or an immediate descendant of it. Then, it resolves a subsequent path from that.
    *
    * @param directoryNameToAssumePresent
    * @param subsequentPath
    */
   public WorkspaceDirectory(String directoryNameToAssumePresent, String subsequentPath)
   {
      Path path = WorkspacePathTools.findPath(directoryNameToAssumePresent, subsequentPath);
      filesystemDirectory = path == null ? null : path.toAbsolutePath();
   }

   /**
    * This constructor uses the Java's {@link java.security.CodeSource} to find the source set directory
    * that contains classForFindingSourceSetDirectory. The resolved path will end in "src/main" or "src/test", etc.
    *
    * @param classForFindingSourceSetDirectory
    */
   public WorkspaceDirectory(Class<?> classForFindingSourceSetDirectory)
   {
      setFilesystemDirectoryToSourceSetDirectory(classForFindingSourceSetDirectory);
   }

   /**
    * This constructor uses the Java's {@link java.security.CodeSource} to find the source set directory
    * that contains classForFindingSourceSetDirectory. The resolved path will end in "src/main" or "src/test", etc.
    * Then, it resolves a subsequent path from that.
    *
    * @param classForFindingSourceSetDirectory
    */
   public WorkspaceDirectory(Class<?> classForFindingSourceSetDirectory, String subsequentPath)
   {
      setFilesystemDirectoryToSourceSetDirectory(classForFindingSourceSetDirectory, subsequentPath);
   }

   /**
    * This is used to create a WorkspaceDirectory when you already know the filesystem
    * directory.
    */
   public WorkspaceDirectory(Path filesystemDirectory)
   {
      this.filesystemDirectory = filesystemDirectory.toAbsolutePath();
   }

   protected void setFilesystemDirectoryToSourceSetDirectory(Class<?> classForFindingSourceSetDirectory)
   {
      Path path = WorkspacePathTools.inferFilesystemSourceSetDirectory(classForFindingSourceSetDirectory);
      filesystemDirectory = path == null ? null : path.toAbsolutePath();
   }

   protected void setFilesystemDirectoryToSourceSetDirectory(Class<?> classForFindingSourceSetDirectory, String subsequentPath)
   {
      setFilesystemDirectoryToSourceSetDirectory(classForFindingSourceSetDirectory);
      if (filesystemDirectory != null)
         filesystemDirectory = filesystemDirectory.resolve(subsequentPath).toAbsolutePath();
   }

   /** If the directory is available for reading/writing using files.
    *  If not, we are running from a JAR without the resource extracted,
    *  or the working directory is wrong. */
   public boolean isFileAccessAvailable()
   {
      return filesystemDirectory != null;
   }

   /**
    * The directory path on the filesystem, if file access is available.
    *
    * Warning: You must check isFileAccessAvailable() first before calling this method!
    *
    * @return The directory Path on the filesystem or null if file access is not available.
    */
   public Path getFilesystemDirectory()
   {
      return filesystemDirectory;
   }

   public WorkspaceFile file(String subsequentPathToFile)
   {
      return new WorkspaceFile(this, subsequentPathToFile);
   }

   public WorkspaceDirectory resolve(String subdirectory)
   {
      if (isFileAccessAvailable())
      {
         return new WorkspaceDirectory(filesystemDirectory.resolve(subdirectory));
      }
      else
      {
         return new WorkspaceDirectory((Path) null);
      }
   }

   /**
    * Used for testing the case where the workspace directory is the src/main source set.
    * To test properly, you must run with the working directory set to repository-group or ihmc-open-robotics-software
    * and also run with the working directory set to ihmc-java-toolkit/src/main.
    */
   public static void main(String[] args)
   {
      WorkspaceDirectory workspaceDirectory = new WorkspaceDirectory("ihmc-open-robotics-software", "ihmc-java-toolkit/src");
      printTestInfo(workspaceDirectory);
   }

   private static void printTestInfo(WorkspaceDirectory workspaceDirectory)
   {
      LogTools.info("File access available: {}", workspaceDirectory.isFileAccessAvailable() ? "Yes" : "No");

      if (workspaceDirectory.isFileAccessAvailable())
      {
         Path directoryPath = workspaceDirectory.getFilesystemDirectory();
         LogTools.info("Directory path: {}", directoryPath);
      }
   }
}
