package us.ihmc.tools.io;

import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.tools.io.resources.ResourceTools;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;

/**
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
 *
 */
public class WorkspaceDirectory
{
   private Class<?> classForLoading;
   private Path workspaceDirectory;
   private String pathNecessaryForClasspathLoading;
   private String pathNecessaryForResourceExploring;

   /**
    * This constructor will infer the correct directory name to assume present and
    * the subsequent path to the resource folder, with even more robustness than passing
    * them in your self.
    */
   public WorkspaceDirectory(Class<?> classForResourceDirectory)
   {
      this(classForResourceDirectory, "");

   }

   /**
    * This constructor will infer the correct directory name to assume present and
    * the subsequent path to the resource folder, with even more robustness than passing
    * them in your self.
    */
   public WorkspaceDirectory(Class<?> classForResourceDirectory, String subsequentOrAbsoluteResourcePackagePath)
   {
      WorkingDirectoryPathComponents workingDirectoryPathComponents = WorkspacePathTools.inferWorkingDirectoryPathComponents(classForResourceDirectory);
      initialize(workingDirectoryPathComponents.getDirectoryNameToAssumePresent(),
                 workingDirectoryPathComponents.getSubsequentPathToResourceFolder(),
                 classForResourceDirectory,
                 subsequentOrAbsoluteResourcePackagePath);
   }

   /**
    * For loading from the root of the resources directory.
    */
   public WorkspaceDirectory(String directoryNameToAssumePresent, String subsequentPathToResourceFolder)
   {
      initialize(directoryNameToAssumePresent, subsequentPathToResourceFolder, null, "");
   }

   public WorkspaceDirectory(String directoryNameToAssumePresent,
                             String subsequentPathToResourceFolder,
                             Class<?> classForResourceDirectory)
   {
      initialize(directoryNameToAssumePresent, subsequentPathToResourceFolder, classForResourceDirectory, "");
   }

   public WorkspaceDirectory(String directoryNameToAssumePresent,
                             String subsequentPathToResourceFolder,
                             String subsequentOrAbsoluteResourcePackagePath)
   {
      initialize(directoryNameToAssumePresent, subsequentPathToResourceFolder, null, subsequentOrAbsoluteResourcePackagePath);
   }

   public WorkspaceDirectory(String directoryNameToAssumePresent,
                             String subsequentPathToResourceFolder,
                             Class<?> classForResourceDirectory,
                             String subsequentOrAbsoluteResourcePackagePath)
   {
      initialize(directoryNameToAssumePresent, subsequentPathToResourceFolder, classForResourceDirectory, subsequentOrAbsoluteResourcePackagePath);
   }

   private void initialize(String directoryNameToAssumePresent,
                           String subsequentPathToResourceFolder,
                           Class<?> classForResourceDirectory,
                           String subsequentOrAbsoluteResourcePackagePath)
   {
      this.classForLoading = classForResourceDirectory;
      String putTogetherResourcePath = "";
      boolean isAbsolute = subsequentOrAbsoluteResourcePackagePath.startsWith("/");
      if (!isAbsolute && classForResourceDirectory != null)
      {
         putTogetherResourcePath += classForResourceDirectory.getPackage().getName().replaceAll("\\.", "/");
         putTogetherResourcePath += "/";
         putTogetherResourcePath += subsequentOrAbsoluteResourcePackagePath;
      }
      else
      {
         if (isAbsolute)
         {
            putTogetherResourcePath += subsequentOrAbsoluteResourcePackagePath.replaceFirst("/", "");
         }
         else
         {
            putTogetherResourcePath += subsequentOrAbsoluteResourcePackagePath;
         }
      }
      pathNecessaryForClasspathLoading = subsequentOrAbsoluteResourcePackagePath;
      String tempPathNecessaryForResourceExploring = pathNecessaryForClasspathLoading;
      if (tempPathNecessaryForResourceExploring.startsWith("/"))
         tempPathNecessaryForResourceExploring = tempPathNecessaryForResourceExploring.replaceFirst("/", "");
      tempPathNecessaryForResourceExploring = tempPathNecessaryForResourceExploring.replaceAll("/", ".");
      pathNecessaryForResourceExploring = tempPathNecessaryForResourceExploring;

      if (directoryNameToAssumePresent == null || subsequentPathToResourceFolder == null)
      {
         workspaceDirectory = null;
      }
      else
      {
         workspaceDirectory = WorkspacePathTools.findPathToResource(directoryNameToAssumePresent,
                                                                    subsequentPathToResourceFolder,
                                                                    putTogetherResourcePath);
      }
   }

   private WorkspaceDirectory(Class<?> classForLoading,
                              Path workspaceDirectory,
                              String pathNecessaryForClasspathLoading,
                              String pathNecessaryForResourceExploring)
   {
      this.classForLoading = classForLoading;
      this.workspaceDirectory = workspaceDirectory;
      this.pathNecessaryForClasspathLoading = pathNecessaryForClasspathLoading;
      this.pathNecessaryForResourceExploring = pathNecessaryForResourceExploring;
   }

   /** If the directory is available for reading/writing using files.
    *  If not, we are running from a JAR without the resource extracted,
    *  or the working directory is wrong. */
   public boolean isFileAccessAvailable()
   {
      return workspaceDirectory != null;
   }

   public void walkResourcesFlat(BiConsumer<String, BasicPathVisitor.PathType> pathVisitor)
   {
      ResourceTools.walkResourcesFlat(pathNecessaryForResourceExploring, pathVisitor);
   }

   public List<WorkspaceFile> queryContainedFiles()
   {
      ArrayList<WorkspaceFile> files = new ArrayList<>();
      ResourceTools.walkResourcesFlat(pathNecessaryForResourceExploring, (fileName, pathType) ->
      {
         if (pathType == BasicPathVisitor.PathType.FILE)
         {
            files.add(new WorkspaceFile(this, fileName));
         }
      });
      return files;
   }

   public Path getDirectoryPath()
   {
      return workspaceDirectory;
   }

   public Class<?> getClassForLoading()
   {
      return classForLoading;
   }

   public String getPathNecessaryForClasspathLoading()
   {
      return pathNecessaryForClasspathLoading;
   }

   public String getPathNecessaryForResourceExploring()
   {
      return pathNecessaryForResourceExploring;
   }

   public WorkspaceFile file(String subsequentPathToFile)
   {
      return new WorkspaceFile(this, subsequentPathToFile);
   }

   public WorkspaceDirectory resolve(String subdirectory)
   {
      return new WorkspaceDirectory(classForLoading,
                                    workspaceDirectory,
                                    pathNecessaryForClasspathLoading + "/" + subdirectory,
                                    pathNecessaryForResourceExploring + "." + subdirectory);
   }
}
