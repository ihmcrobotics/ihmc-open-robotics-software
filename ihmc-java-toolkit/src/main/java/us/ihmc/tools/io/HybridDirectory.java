package us.ihmc.tools.io;

import java.nio.file.Path;

/**
 * A class used for managing a resources that you're either using from source
 * version control or using a version somewhere else local to your machine.
 *
 * This class provides an easily switchable "mode" between accessing the workspace
 * or external version of this directory.
 *
 * A hybrid directory defines:
 * - Workspace directory: A directory in your source code workspace checkout in a resources source set
 * - External directory: A directory somewhere on your file system not in your workspace
 *
 * Example:
 * Workspace: ihmc-open-robotics-software/ihmc-java-toolkit/stuff
 * External: ~/.ihmc/stuff
 */
public class HybridDirectory
{
   protected Path externalDirectory;
   protected WorkspaceDirectory workspaceDirectory;
   protected HybridResourceMode mode = HybridResourceMode.WORKSPACE;

   protected HybridDirectory()
   {

   }

   public HybridDirectory(Path externalDirectory, Path workspaceDirectory)
   {
      this.externalDirectory = externalDirectory;
      this.workspaceDirectory = new WorkspaceDirectory(workspaceDirectory);
   }

   public HybridDirectory(Path externalDirectory, WorkspaceDirectory workspaceDirectory)
   {
      this.externalDirectory = externalDirectory;
      this.workspaceDirectory = workspaceDirectory;
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
      return mode == HybridResourceMode.WORKSPACE ? workspaceDirectory.getFilesystemDirectory() : externalDirectory;
   }

   public Path getWorkspaceDirectory()
   {
      return workspaceDirectory.getFilesystemDirectory();
   }

   public Path getExternalDirectory()
   {
      return externalDirectory;
   }

   public HybridDirectory resolve(String subsequentPathInBothExternalAndWorkspace)
   {
      return new HybridDirectory(externalDirectory.resolve(subsequentPathInBothExternalAndWorkspace),
                                 workspaceDirectory.resolve(subsequentPathInBothExternalAndWorkspace));
   }

   /* package private */ WorkspaceDirectory getWorkspaceDirectoryInternal()
   {
      return workspaceDirectory;
   }
}
