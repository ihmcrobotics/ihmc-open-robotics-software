package us.ihmc.tools.io.files;

import java.nio.file.FileVisitResult;
import java.nio.file.Path;

/**
 * A more convenient interface for walking file trees. Sits on top of Java's NIO.2.
 */
public abstract class BasicPathVisitor
{
   /**
    * A file or a directory.
    */
   public enum PathType
   {
      FILE,
      DIRECTORY,
   }
   
   /**
    * This method is called when a Path is visited.
    * 
    * @param path the Path being visited
    * @param pathType the type of Path being visited (file or directory) 
    * @return fileVisitResult CONTINUE, SKIP_SIBLINGS, SKIP_SUBTREE, or TERMINATE
    */
   public FileVisitResult visitPath(Path path, PathType pathType)
   {
      return FileVisitResult.CONTINUE;
   }
}
