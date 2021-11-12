package us.ihmc.tools.io;

import java.nio.file.Path;

public class WorkspaceDirectory
{
   private final Class<?> classForLoading;
   private final Path workspaceDirectory;
   private final String pathNecessaryForClasspathLoading;

   public WorkspaceDirectory(String directoryNameToAssumePresent,
                             String subsequentPathToResourceFolder,
                             Class<?> classForResourceDirectory)
   {
      this(directoryNameToAssumePresent, subsequentPathToResourceFolder, classForResourceDirectory, "");
   }

   public WorkspaceDirectory(String directoryNameToAssumePresent,
                             String subsequentPathToResourceFolder,
                             Class<?> classForResourceDirectory,
                             String subsequentOrAbsoluteResourcePackagePath)
   {
      String subsequentExternalPath = subsequentOrAbsoluteResourcePackagePath;
      if (subsequentExternalPath.startsWith("/"))
         subsequentExternalPath = subsequentExternalPath.replaceFirst("/", "");

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
}
