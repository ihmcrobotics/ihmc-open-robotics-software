package us.ihmc.tools.io;

import java.nio.file.Path;

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
         pathNecessaryForClasspathLoading = subsequentOrAbsoluteResourcePackagePath;
      }
      else if (isAbsolute)
      {
         putTogetherResourcePath += subsequentOrAbsoluteResourcePackagePath.replaceFirst("/", "");
         pathNecessaryForClasspathLoading = subsequentOrAbsoluteResourcePackagePath;
      }
      else // class is null & path is relative
      {
         pathNecessaryForClasspathLoading = subsequentOrAbsoluteResourcePackagePath;
      }

      workspaceDirectory = WorkspacePathTools.findPathToResource(directoryNameToAssumePresent,
                                                                 subsequentPathToResourceFolder,
                                                                 putTogetherResourcePath);
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
}
