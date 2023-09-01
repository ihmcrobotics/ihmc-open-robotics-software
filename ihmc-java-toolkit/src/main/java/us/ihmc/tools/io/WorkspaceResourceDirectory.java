package us.ihmc.tools.io;

import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.resources.ResourceTools;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;

/**
 * Represents a resource directory or subdirectory on the classpath and in your source version
 * controlled workspace.
 *
 * See documentation on {@link WorkspaceDirectory}, which this extends. The same concepts apply.
 */
public class WorkspaceResourceDirectory extends WorkspaceDirectory
{
   protected Class<?> classForLoading;
   /** Classpath path with slashes. */
   private final Path pathNecessaryForClasspathLoading;
   private String pathNecessaryForClasspathLoadingString;
   /** Classpath path with dots. */
   private String pathNecessaryForResourceExploring;

   /**
    * This constructor will get a path to the resources directory for a class.
    *
    * Example, pass in the class us.ihmc.tools.SomeClass will result in:
    * /path/to/ihmc-open-robotics-software/ihmc-java-toolkit/src/main/resources/us/ihmc/tools
    */
   public WorkspaceResourceDirectory(Class<?> classForFindingSourceSetDirectory)
   {
      this.classForLoading = classForFindingSourceSetDirectory;
      setFilesystemDirectoryToSourceSetDirectory(classForFindingSourceSetDirectory, "resources");
      pathNecessaryForClasspathLoading = ResourceTools.getResourcesPathForClass(classForFindingSourceSetDirectory);
      initialize();
   }

   /**
    * This constructor will get a path to the resources directory for a class, with the option to specify
    * a descendant directory of that or an ancestor from the classpath root.
    *
    * Example, pass in the class us.ihmc.tools.SomeClass, and pass "stuff/moreStuff" will result in:
    * /path/to/ihmc-open-robotics-software/ihmc-java-toolkit/src/main/resources/us/ihmc/tools/stuff/moreStuff
    *
    * Example, pass in the class us.ihmc.tools.SomeClass, and pass "/stuff/moreStuff" will result in:
    * /path/to/ihmc-open-robotics-software/ihmc-java-toolkit/src/main/resources/stuff/moreStuff
    */
   public WorkspaceResourceDirectory(Class<?> classForFindingSourceSetDirectory, String subsequentOrAbsoluteResourcePackagePath)
   {
      this.classForLoading = classForFindingSourceSetDirectory;
      setFilesystemDirectoryToSourceSetDirectory(classForFindingSourceSetDirectory, "resources");
      pathNecessaryForClasspathLoading = ResourceTools.getResourcesPathForClass(classForFindingSourceSetDirectory)
                                                      .resolve(subsequentOrAbsoluteResourcePackagePath);
      initialize();
   }

   private void initialize()
   {
      pathNecessaryForClasspathLoadingString = ResourceTools.toResourceAccessStringWithCorrectSeparators(pathNecessaryForClasspathLoading);
      // This path isn't absolute for the filesystem, only for the classpath, so we remove the first
      // character which is ensured to be a forward slash ('/') at this point.
      // i.e. /us/ihmc/tools/io would need to be us/ihmc/tools/io
      // to append to /path/to/ihmc-java-toolkit/src/test/resources/us/ihmc/tools/io
      String classpathPathToResourceDirectoryString = pathNecessaryForClasspathLoadingString.substring(1);
      if (filesystemDirectory != null)
      {
         filesystemDirectory = filesystemDirectory.resolve(classpathPathToResourceDirectoryString);
      }
      pathNecessaryForResourceExploring = classpathPathToResourceDirectoryString.replaceAll("/", ".");
   }

   public void walkResourcesFlat(BiConsumer<String, BasicPathVisitor.PathType> pathVisitor)
   {
      ResourceTools.walkResourcesFlat(pathNecessaryForResourceExploring, pathVisitor);
   }

   public List<WorkspaceResourceFile> queryContainedFiles()
   {
      ArrayList<WorkspaceResourceFile> files = new ArrayList<>();
      ResourceTools.walkResourcesFlat(pathNecessaryForResourceExploring, (fileName, pathType) ->
      {
         if (pathType == BasicPathVisitor.PathType.FILE)
         {
            files.add(new WorkspaceResourceFile(this, fileName));
         }
      });
      return files;
   }

   /**
    * For use with {@link ClassLoader#getResourceAsStream}.
    */
   public Class<?> getClassForLoading()
   {
      return classForLoading;
   }

   /**
    * For use with {@link ClassLoader#getResourceAsStream} and {@link ClassLoader#getSystemResourceAsStream(String)}, etc.
    * @return a package path with slashes, beginning with a slash if absolute i.e. "/us/ihmc/tools" or "ihmc/tools"
    */
   public String getPathNecessaryForClasspathLoading()
   {
      return pathNecessaryForClasspathLoadingString;
   }

   /**
    * For use with {@link us.ihmc.tools.io.resources.ResourceTools#listResources} to list
    * the resources available in a directory.
    * @return a package path with dots i.e. "us.ihmc.tools"
    */
   public String getPathNecessaryForResourceExploring()
   {
      return pathNecessaryForResourceExploring;
   }

   public WorkspaceResourceDirectory resolve(String subdirectory)
   {
      return new WorkspaceResourceDirectory(classForLoading,
               ResourceTools.toResourceAccessStringWithCorrectSeparators(pathNecessaryForClasspathLoading.resolve(subdirectory)));
   }

   /**
    * Used for testing the case where the workspace directory is the src/main source set.
    * To test properly, you must run with the working directory set to repository-group or ihmc-open-robotics-software
    * and also run with the working directory set to ihmc-java-toolkit/src/main.
    */
   public static void main(String[] args)
   {
      WorkspaceResourceDirectory workspaceDirectory = new WorkspaceResourceDirectory(WorkspaceResourceDirectory.class);
      printTestInfo(workspaceDirectory);
      workspaceDirectory = new WorkspaceResourceDirectory(WorkspaceResourceDirectory.class, "/stuff/moreStuff");
      printTestInfo(workspaceDirectory);
      workspaceDirectory = new WorkspaceResourceDirectory(WorkspaceResourceDirectory.class, "stuff/moreStuff");
      printTestInfo(workspaceDirectory);
   }

   private static void printTestInfo(WorkspaceResourceDirectory workspaceDirectory)
   {
      LogTools.info("File access available: {}", workspaceDirectory.isFileAccessAvailable() ? "Yes" : "No");
      LogTools.info("Path necessary for classpath loading: {}", workspaceDirectory.getPathNecessaryForClasspathLoading());
      LogTools.info("Path necessary for resource exploring: {}", workspaceDirectory.getPathNecessaryForResourceExploring());

      if (workspaceDirectory.isFileAccessAvailable())
      {
         Path directoryPath = workspaceDirectory.getFilesystemDirectory();
         LogTools.info("Directory path: {}", directoryPath);
      }
   }
}
