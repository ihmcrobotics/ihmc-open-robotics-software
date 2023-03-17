package us.ihmc.tools.io;

import java.nio.file.Path;

/**
 * Represents the absolute path to a "source set" on the filesystem, like "/path/to/project/src/main"
 * or "/path/to/project/src/test".
 */
public class FilesystemSourceSetDirectory
{
   private final Path parentOfSrcDirectory;
   private final Path subsequentPathToSourceSet;

   public FilesystemSourceSetDirectory(Path parentOfSrcDirectory, Path subsequentPathToSourceSet)
   {
      this.parentOfSrcDirectory = parentOfSrcDirectory;
      this.subsequentPathToSourceSet = subsequentPathToSourceSet;
   }

   /**
    * @return The full absolute filesystem path to the project directory, which is strictly
    * the parent of the "src" directory that contains the Java source file.
    */
   public Path getParentOfSrcDirectory()
   {
      return parentOfSrcDirectory;
   }

   /**
    * @return The subsequent path to the source set. i.e. "src/main" or "src/test".
    */
   public Path getSubsequentPathToSourceSet()
   {
      return subsequentPathToSourceSet;
   }
}
