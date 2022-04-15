package us.ihmc.tools.io;

import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.tools.io.resources.ResourceTools;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;

public class WorkspaceDirectory
{
   private final Class<?> classForLoading;
   private final Path workspaceDirectory;
   private final String pathNecessaryForClasspathLoading;
   private final String pathNecessaryForResourceExploring;

   /**
    * For loading from the root of the resources directory.
    */
   public WorkspaceDirectory(String directoryNameToAssumePresent, String subsequentPathToResourceFolder)
   {
      this(directoryNameToAssumePresent, subsequentPathToResourceFolder, null, "");
   }

   public WorkspaceDirectory(String directoryNameToAssumePresent,
                             String subsequentPathToResourceFolder,
                             Class<?> classForResourceDirectory)
   {
      this(directoryNameToAssumePresent, subsequentPathToResourceFolder, classForResourceDirectory, "");

   }
   public WorkspaceDirectory(String directoryNameToAssumePresent,
                             String subsequentPathToResourceFolder,
                             String subsequentOrAbsoluteResourcePackagePath)
   {
      this(directoryNameToAssumePresent, subsequentPathToResourceFolder, null, subsequentOrAbsoluteResourcePackagePath);
   }

   public WorkspaceDirectory(String directoryNameToAssumePresent,
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

      workspaceDirectory = WorkspacePathTools.findPathToResource(directoryNameToAssumePresent,
                                                                 subsequentPathToResourceFolder,
                                                                 putTogetherResourcePath);
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
}
