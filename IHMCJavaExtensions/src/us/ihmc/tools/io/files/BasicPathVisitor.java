package us.ihmc.tools.io.files;

import java.nio.file.FileVisitResult;
import java.nio.file.Path;

public abstract class BasicPathVisitor
{
   public enum PathType
   {
      FILE,
      DIRECTORY,
   }
   
   public FileVisitResult visitPath(Path path, PathType pathType)
   {
      return FileVisitResult.CONTINUE;
   }
}
