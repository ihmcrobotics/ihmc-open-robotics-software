package us.ihmc.tools.io;

import java.nio.file.Path;

public class WorkingDirectoryPathComponents
{
   private final Path parentOfSrcDirectory;
   private final Path subsequentPathToResourceFolder;

   public WorkingDirectoryPathComponents(Path parentOfSrcDirectory, Path subsequentPathToResourceFolder)
   {
      this.parentOfSrcDirectory = parentOfSrcDirectory;
      this.subsequentPathToResourceFolder = subsequentPathToResourceFolder;
   }

   public Path getParentOfSrcDirectory()
   {
      return parentOfSrcDirectory;
   }

   public Path getSubsequentPathToSourceSet()
   {
      return subsequentPathToResourceFolder;
   }
}
