package us.ihmc.tools.io;

import us.ihmc.commons.nio.BasicPathVisitor;

import java.nio.file.Path;
import java.util.function.BiConsumer;

/**
 * See {@link HybridDirectory}. This class extends that to provide more assistance for
 * when the directory is in the resources classpath.
 */
public class HybridResourceDirectory extends HybridDirectory
{
   private final WorkspaceResourceDirectory workspaceResourceDirectory;

   public HybridResourceDirectory(Path externalDirectory, Class<?> classForResourceDirectory)
   {
      this.externalDirectory = externalDirectory;
      workspaceResourceDirectory = new WorkspaceResourceDirectory(classForResourceDirectory);
      workspaceDirectory = workspaceResourceDirectory;
   }

   public HybridResourceDirectory(Path externalDirectory, Class<?> classForWorkspaceResourceDirectory, String workspaceSubsequentOrAbsoluteResourcePackagePath)
   {
      this.externalDirectory = externalDirectory;
      workspaceResourceDirectory = new WorkspaceResourceDirectory(classForWorkspaceResourceDirectory, workspaceSubsequentOrAbsoluteResourcePackagePath);
      workspaceDirectory = workspaceResourceDirectory;
   }

   private HybridResourceDirectory(Path externalDirectory, WorkspaceResourceDirectory workspaceResourceDirectory)
   {
      this.externalDirectory = externalDirectory;
      this.workspaceResourceDirectory = workspaceResourceDirectory;
      this.workspaceDirectory = workspaceResourceDirectory;
   }

   public void walkResourcesFlat(BiConsumer<String, BasicPathVisitor.PathType> pathVisitor)
   {
      workspaceResourceDirectory.walkResourcesFlat(pathVisitor);
   }

   public Class<?> getClassForLoading()
   {
      return workspaceResourceDirectory.getClassForLoading();
   }

   public String getPathNecessaryForClasspathLoading()
   {
      return workspaceResourceDirectory.getPathNecessaryForClasspathLoading();
   }

   public String getPathNecessaryForResourceExploring()
   {
      return workspaceResourceDirectory.getPathNecessaryForResourceExploring();
   }

   public HybridResourceDirectory resolve(String subsequentPathInBothExternalAndWorkspace)
   {
      return new HybridResourceDirectory(externalDirectory.resolve(subsequentPathInBothExternalAndWorkspace),
                                         workspaceResourceDirectory.resolve(subsequentPathInBothExternalAndWorkspace));
   }

   /* package private */ WorkspaceResourceDirectory getWorkspaceResourceDirectoryInternal()
   {
      return workspaceResourceDirectory;
   }
}
