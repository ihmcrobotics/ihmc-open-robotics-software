package us.ihmc.tools.io;

import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.tools.io.resources.ResourceTools;

import java.nio.file.Path;
import java.util.TreeSet;
import java.util.function.BiConsumer;

/**
 * A hybrid directory defines:
 * - Workspace directory: A directory in your source code workspace checkout in a resources source set
 * - External directory: A directory somewhere on your file system not in your workspace
 */
public class HybridDirectory
{
   private final Path externalDirectory;
   private final Class<?> classForLoading;
   private final Path workspaceDirectory;
   private final String pathNecessaryForClasspathLoading;
   private final String pathNecessaryForResourceExploring;
   private HybridResourceMode mode = HybridResourceMode.WORKSPACE;

   public HybridDirectory(Path externalDirectory,
                          String directoryNameToAssumePresent,
                          String subsequentPathToResourceFolder,
                          Class<?> classForResourceDirectory)
   {
      this(externalDirectory,
           directoryNameToAssumePresent,
           subsequentPathToResourceFolder,
           classForResourceDirectory,
           "");
   }

   /**
    * @deprecated This is broken for some reason.
    */
   public HybridDirectory(Path externalDirectory,
                          String directoryNameToAssumePresent,
                          String subsequentPathToResourceFolder,
                          String absoluteResourceDirectory)
   {
      this(externalDirectory,
           directoryNameToAssumePresent,
           subsequentPathToResourceFolder,
           null,
           absoluteResourceDirectory);
   }

   public HybridDirectory(Path externalDirectory,
                          String directoryNameToAssumePresent,
                          String subsequentPathToResourceFolder,
                          Class<?> classForResourceDirectory,
                          String subsequentOrAbsoluteResourcePackagePath)
   {
      String subsequentExternalPath = subsequentOrAbsoluteResourcePackagePath;
      if (subsequentExternalPath.startsWith("/"))
         subsequentExternalPath = subsequentExternalPath.replaceFirst("/", "");
      this.externalDirectory = externalDirectory.resolve(subsequentExternalPath).toAbsolutePath().normalize();

      this.classForLoading = classForResourceDirectory;
      String putTogetherResourcePath = "";
      boolean isAbsolute = subsequentOrAbsoluteResourcePackagePath.startsWith("/");
      if (!isAbsolute && classForResourceDirectory != null)
      {
         putTogetherResourcePath += classForResourceDirectory.getPackage().getName().replaceAll("\\.", "/");
         putTogetherResourcePath += "/";
         putTogetherResourcePath += subsequentOrAbsoluteResourcePackagePath;
      }
      else if (isAbsolute)
      {
         putTogetherResourcePath += subsequentOrAbsoluteResourcePackagePath.replaceFirst("/", "");
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

   public void walkResourcesFlat(BiConsumer<String, BasicPathVisitor.PathType> pathVisitor)
   {
      TreeSet<String> fileNames = new TreeSet<>();
      TreeSet<String> directoryNames = new TreeSet<>();
      for (String resourceEntry : ResourceTools.listResources(pathNecessaryForResourceExploring, ".*"))
      {
         if (resourceEntry.contains("/"))
         {
            directoryNames.add(resourceEntry.substring(0, resourceEntry.indexOf("/")));
         }
         else
         {
            fileNames.add(resourceEntry);
         }
      }
      for (String fileName : fileNames)
      {
         pathVisitor.accept(fileName, BasicPathVisitor.PathType.FILE);
      }
      for (String directoryName : directoryNames)
      {
         pathVisitor.accept(directoryName, BasicPathVisitor.PathType.DIRECTORY);
      }
   }

   /**
    * i.e. Cannot write to resource directories inside JARs
    */
   public boolean isWorkspaceWritingAvailable()
   {
      return workspaceDirectory != null;
   }

   public void setMode(HybridResourceMode mode)
   {
      this.mode = mode;
   }

   public HybridResourceMode getMode()
   {
      return mode;
   }

   public Path getDirectoryForWriting()
   {
      return mode == HybridResourceMode.WORKSPACE ? workspaceDirectory : externalDirectory;
   }

   public Path getWorkspaceDirectory()
   {
      return workspaceDirectory;
   }

   public Path getExternalDirectory()
   {
      return externalDirectory;
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
