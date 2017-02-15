package us.ihmc.tools.io.files;

import java.io.IOException;
import java.nio.file.FileSystems;
import java.nio.file.FileVisitOption;
import java.nio.file.FileVisitResult;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.PathMatcher;
import java.nio.file.SimpleFileVisitor;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;

import org.apache.commons.io.FilenameUtils;

import us.ihmc.tools.io.files.BasicPathVisitor.PathType;

public class PathTools
{
   private static final String GLOB_SYNTAX_PREFIX = "glob:";
   private static final String REGEX_SYNTAX_PREFIX = "regex:";
   
   public static List<Path> findAllPathsRecursivelyThatMatchRegex(Path rootPath, String regex)
   {
      final PathMatcher matcher = FileSystems.getDefault().getPathMatcher(REGEX_SYNTAX_PREFIX + regex);
      final List<Path> matchingPaths = new ArrayList<Path>();
      
      walkRecursively(rootPath, new BasicPathVisitor()
      {
         @Override
         public FileVisitResult visitPath(Path path, PathType pathType)
         {
            if (matcher.matches(path))
               matchingPaths.add(path);
            
            return FileVisitResult.CONTINUE;
         }
      });
      
      return matchingPaths;
   }
   
   public static boolean contains(Path path, String name)
   {
      for (int i = 0; i < path.getNameCount(); i++)
      {
         if (path.getName(i).toString().equals(name))
         {
            return true;
         }
      }

      return false;
   }

   public static String getBaseName(Path path)
   {
      return FilenameUtils.getBaseName(path.toString());
   }
   
   public static String getExtension(Path path)
   {
      return FilenameUtils.getExtension(path.toString());
   }

   public static Path findFirstPathMatchingGlob(Path directory, final String glob)
   {      
      final PathMatcher matcher = FileSystems.getDefault().getPathMatcher(GLOB_SYNTAX_PREFIX + glob);
      final Path[] pathHolder = {null};
      
      walkRecursively(directory, new BasicPathVisitor()
      {         
         @Override
         public FileVisitResult visitPath(Path path, PathType pathType)
         {
            if (matcher.matches(path))
            {
               pathHolder[0] = path;
               
               return FileVisitResult.TERMINATE;
            }
            
            return FileVisitResult.CONTINUE;
         }
      });

      return pathHolder[0];
   }

   public static boolean directoryHasGlob(Path directory, final String glob)
   {
      return findFirstPathMatchingGlob(directory, glob) != null;
   }

   public static void walkRecursively(Path directory, final BasicPathVisitor basicFileVisitor)
   {
      try
      {
         Files.walkFileTree(directory, new SimpleFileVisitor<Path>()
         {
            @Override
            public FileVisitResult preVisitDirectory(Path dir, BasicFileAttributes attrs) throws IOException
            {
               return basicFileVisitor.visitPath(dir, PathType.DIRECTORY);
            }
            
            @Override
            public FileVisitResult visitFile(Path file, BasicFileAttributes attrs) throws IOException
            {
               return basicFileVisitor.visitPath(file, PathType.FILE);
            }
         });
      }
      catch (IOException e)
      {
      }
   }
   
   public static void walkDepth(final Path directory, int maxDepth, final BasicPathVisitor basicFileVisitor)
   {
      try
      {
         Files.walkFileTree(directory, EnumSet.noneOf(FileVisitOption.class), maxDepth, new SimpleFileVisitor<Path>()
         {
            @Override
            public FileVisitResult preVisitDirectory(Path dir, BasicFileAttributes attrs) throws IOException
            {
               if (dir.equals(directory))
                  return FileVisitResult.CONTINUE;
               
               return basicFileVisitor.visitPath(dir, PathType.DIRECTORY);
            }
            
            @Override
            public FileVisitResult visitFile(Path file, BasicFileAttributes attrs) throws IOException
            {
               if (Files.isDirectory(file))
               {
                  return basicFileVisitor.visitPath(file, PathType.DIRECTORY);
               }
               {
                  return basicFileVisitor.visitPath(file, PathType.FILE);
               }
            }
         });
      }
      catch (IOException e)
      {
      }
   }

   public static void walkFlat(final Path directory, final BasicPathVisitor basicFileVisitor)
   {
      walkDepth(directory, 1, basicFileVisitor);
   }
}
