package us.ihmc.tools.io;

import us.ihmc.log.LogTools;
import us.ihmc.tools.io.resources.ResourceTools;

import java.nio.file.Path;

/**
 * Represents a resource directory or subdirectory containing Java source files.
 *
 * See documentation on {@link WorkspaceDirectory}, which this extends. The same concepts apply.
 */
public class WorkspaceJavaDirectory extends WorkspaceDirectory
{
   private final Path classpathPath;

   /**
    * This constructor will get a path to the java directory containing this class.
    *
    * Example, pass in the class us.ihmc.tools.SomeClass will result in:
    * /path/to/ihmc-open-robotics-software/ihmc-java-toolkit/src/main/java/us/ihmc/tools
    */
   public WorkspaceJavaDirectory(Class<?> classForFindingSourceSetDirectory, String javaDirectoryName)
   {
      setFilesystemDirectoryToSourceSetDirectory(classForFindingSourceSetDirectory, javaDirectoryName);
      classpathPath = ResourceTools.getResourcesPathForClass(classForFindingSourceSetDirectory);
      initialize();
   }

   /**
    * This constructor will get a path to the java directory for a class, with the option to specify
    * a descendant directory of that or an ancestor from the classpath root.
    *
    * Example, pass in the class us.ihmc.tools.SomeClass, and pass "stuff/moreStuff" will result in:
    * /path/to/ihmc-open-robotics-software/ihmc-java-toolkit/src/main/java/us/ihmc/tools/stuff/moreStuff
    *
    * Example, pass in the class us.ihmc.tools.SomeClass, and pass "/stuff/moreStuff" will result in:
    * /path/to/ihmc-open-robotics-software/ihmc-java-toolkit/src/main/java/stuff/moreStuff
    */
   public WorkspaceJavaDirectory(Class<?> classForFindingSourceSetDirectory, String javaDirectoryName, String subsequentOrAbsoluteJavaPackagePath)
   {
      setFilesystemDirectoryToSourceSetDirectory(classForFindingSourceSetDirectory, javaDirectoryName);
      classpathPath = ResourceTools.getResourcesPathForClass(classForFindingSourceSetDirectory).resolve(subsequentOrAbsoluteJavaPackagePath);
      initialize();
   }

   private void initialize()
   {
      // This path isn't absolute for the filesystem, only for the classpath
      // i.e. /us/ihmc/tools/io would need to be us/ihmc/tools/io
      // to append to /path/to/ihmc-java-toolkit/src/test/resources/us/ihmc/tools/io
      String subsequentPathToJavaDirectory = classpathPath.toString().substring(1);
      if (filesystemDirectory != null)
         filesystemDirectory = filesystemDirectory.resolve(subsequentPathToJavaDirectory);
   }

   /**
    * Used for testing the case where the workspace directory is the src/main source set.
    * To test properly, you must run with the working directory set to repository-group or ihmc-open-robotics-software
    * and also run with the working directory set to ihmc-java-toolkit/src/main.
    */
   public static void main(String[] args)
   {
      WorkspaceJavaDirectory workspaceDirectory = new WorkspaceJavaDirectory(WorkspaceJavaDirectory.class, "java");
      printTestInfo(workspaceDirectory);
      workspaceDirectory = new WorkspaceJavaDirectory(WorkspaceJavaDirectory.class, "java", "/stuff/moreStuff");
      printTestInfo(workspaceDirectory);
      workspaceDirectory = new WorkspaceJavaDirectory(WorkspaceJavaDirectory.class, "java", "stuff/moreStuff");
      printTestInfo(workspaceDirectory);
   }

   private static void printTestInfo(WorkspaceJavaDirectory workspaceDirectory)
   {
      LogTools.info("File access available: {}", workspaceDirectory.isFileAccessAvailable() ? "Yes" : "No");

      if (workspaceDirectory.isFileAccessAvailable())
      {
         Path directoryPath = workspaceDirectory.getFilesystemDirectory();
         LogTools.info("Directory path: {}", directoryPath);
      }
   }
}
