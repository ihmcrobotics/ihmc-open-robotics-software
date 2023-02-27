package us.ihmc.tools.io;

import us.ihmc.commons.nio.BasicPathVisitor;

import java.nio.file.Path;
import java.util.function.BiConsumer;

/**
 * A hybrid directory defines:
 * - Workspace directory: A directory in your source code workspace checkout in a resources source set
 * - External directory: A directory somewhere on your file system not in your workspace
 */
public class HybridDirectory
{
   private final WorkspaceDirectory workspaceDirectory;
   private final Path externalDirectory;
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
      this.workspaceDirectory = new WorkspaceDirectory(directoryNameToAssumePresent,
                                                       subsequentPathToResourceFolder,
                                                       classForResourceDirectory,
                                                       subsequentOrAbsoluteResourcePackagePath);

      String subsequentExternalPath = subsequentOrAbsoluteResourcePackagePath;
      if (subsequentExternalPath.startsWith("/"))
         subsequentExternalPath = subsequentExternalPath.replaceFirst("/", "");
      this.externalDirectory = externalDirectory.resolve(subsequentExternalPath).toAbsolutePath().normalize();
   }

   public void walkResourcesFlat(BiConsumer<String, BasicPathVisitor.PathType> pathVisitor)
   {
      workspaceDirectory.walkResourcesFlat(pathVisitor);
   }

   /** If the directory is available for reading/writing using files.
    *  If not, we are running from a JAR without the resource extracted,
    *  or the working directory is wrong. */
   public boolean isWorkspaceFileAccessAvailable()
   {
      return workspaceDirectory.isFileAccessAvailable();
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
      return mode == HybridResourceMode.WORKSPACE ? workspaceDirectory.getDirectoryPath() : externalDirectory;
   }

   public Path getWorkspaceDirectory()
   {
      return workspaceDirectory.getDirectoryPath();
   }

   public Path getExternalDirectory()
   {
      return externalDirectory;
   }

   public Class<?> getClassForLoading()
   {
      return workspaceDirectory.getClassForLoading();
   }

   public String getPathNecessaryForClasspathLoading()
   {
      return workspaceDirectory.getPathNecessaryForClasspathLoading();
   }

   public String getPathNecessaryForResourceExploring()
   {
      return workspaceDirectory.getPathNecessaryForResourceExploring();
   }

   WorkspaceDirectory getInternalWorkspaceDirectory()
   {
      return workspaceDirectory;
   }
}
