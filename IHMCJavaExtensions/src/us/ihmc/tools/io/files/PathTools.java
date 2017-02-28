package us.ihmc.tools.io.files;

import java.io.IOException;
import java.nio.file.FileSystems;
import java.nio.file.FileVisitOption;
import java.nio.file.FileVisitResult;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.PathMatcher;
import java.nio.file.Paths;
import java.nio.file.SimpleFileVisitor;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;

import org.apache.commons.io.FilenameUtils;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.tools.io.files.BasicPathVisitor.PathType;

/**
 * <p>A collection of tools to extend Java's NIO.2 API and
 * Apache Commons Lang. Tools here should fit one of
 * the following categories:</p>
 * 
 * <ol>Provide a commonly needed method not provided by Apache Commons Lang or Java's NIO.2. API.</ol>
 * <ol>Provide a wrapper around a commonly used method that uses a {@link DefaultExceptionHandler}.</ol>
 * <ol>Provide a bridge between Java's NIO.2 API and Apache Commons Lang.</ol>
 */
public class PathTools
{
   private static final String GLOB_SYNTAX_PREFIX = "glob:";
   private static final String REGEX_SYNTAX_PREFIX = "regex:";
   
   /**
    * Get the base name of a file. A bridge from Java's NIO.2 to Apache Commons IO.
    * 
    * @param path path
    * @return baseName the base name, minus the full path and extension, from a full filename
    */
   public static String getBaseName(Path path)
   {
      return FilenameUtils.getBaseName(path.toString());
   }
   
   /**
    * Get the extension of a file. A bridge from Java's NIO.2 to Apache Commons IO.
    * 
    * @param path path
    * @return extension the extension of a file name
    */
   public static String getExtension(Path path)
   {
      return FilenameUtils.getExtension(path.toString());
   }

   public static Path systemTemporaryDirectory()
   {
      return Paths.get(System.getProperty("java.io.tmpdir"));
   }

   /**
    * Find a list of all Paths that match regex.
    * 
    * @see {@link java.util.regex.Pattern}
    * 
    * @param directory directory to search
    * @param regex regular expression as defined by {@link java.util.regex.Pattern}
    * @return List of matching Paths.
    */
   public static List<Path> findAllPathsRecursivelyThatMatchRegex(Path directory, String regex)
   {
      final PathMatcher matcher = FileSystems.getDefault().getPathMatcher(REGEX_SYNTAX_PREFIX + regex);
      final List<Path> matchingPaths = new ArrayList<Path>();
      
      walkRecursively(directory, new BasicPathVisitor()
      {
         @Override
         public FileVisitResult visitPath(Path path, PathType pathType)
         {
            if (matcher.matches(path))
            {
               matchingPaths.add(path);
            }
            
            return FileVisitResult.CONTINUE;
         }
      });
      
      return matchingPaths;
   }

   /**
    * Find the first Path that matches the glob.
    * 
    * @see {@link PathMatcher}
    * 
    * @param directory directory to search
    * @param glob glob as defined by {@link PathMatcher}
    * @return List of matching Paths.
    */
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

   /**
    * Determines if there is a file or directory that matches <code>glob</code>.
    * 
    * @see {@link PathMatcher}
    * 
    * @param directory directory to search
    * @param glob glob as defined by {@link PathMatcher}
    * @return Has glob boolean.
    */
   public static boolean directoryHasGlob(Path directory, final String glob)
   {
      return findFirstPathMatchingGlob(directory, glob) != null;
   }

   /**
    * Recursively walk through a directory. A simple case of Files.walkFileTree provided by Java's NIO.2.
    * 
    * <p>WARNING: This method is best try only. All exceptions will be swallowed silently. For more specific behavior
    * you must use {@link Files#walkFileTree(Path, FileVisitor)} directly.</p>
    * 
    * @param directory directory to walk
    * @param basicFileVisitor callback to take action on visits
    */
   public static void walkRecursively(Path directory, final BasicPathVisitor basicFileVisitor)
   {
      try
      {
         Files.walkFileTree(directory, new SimpleFileVisitor<Path>()
         {
            @Override
            public FileVisitResult preVisitDirectory(Path directory, BasicFileAttributes attributes) throws IOException
            {
               return basicFileVisitor.visitPath(directory, PathType.DIRECTORY);
            }
            
            @Override
            public FileVisitResult visitFile(Path file, BasicFileAttributes attributes) throws IOException
            {
               return basicFileVisitor.visitPath(file, PathType.FILE);
            }
         });
      }
      catch (IOException e)
      {
      }
   }
   
   /**
    * <p>Walk through a directory to a max depth. A simple case of Files.walkFileTree provided by Java's NIO.2.</p>
    * 
    * <p>WARNING: This method is best try only. All exceptions will be swallowed silently. For more specific behavior
    * you must use {@link Files#walkFileTree(Path, FileVisitor)} directly.</p>
    * 
    * @param directory directory to walk
    * @param basicFileVisitor callback to take action on visits
    */
   public static void walkDepth(final Path directory, int maxDepth, final BasicPathVisitor basicFileVisitor)
   {
      try
      {
         Files.walkFileTree(directory, EnumSet.noneOf(FileVisitOption.class), maxDepth, new SimpleFileVisitor<Path>()
         {
            @Override
            public FileVisitResult preVisitDirectory(Path directory, BasicFileAttributes attributes) throws IOException
            {
               if (directory.equals(directory))
               {
                  return FileVisitResult.CONTINUE;
               }
               
               return basicFileVisitor.visitPath(directory, PathType.DIRECTORY);
            }
            
            @Override
            public FileVisitResult visitFile(Path file, BasicFileAttributes attributes) throws IOException
            {
               if (Files.isDirectory(file))
               {
                  return basicFileVisitor.visitPath(file, PathType.DIRECTORY);
               }
               else
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

   /**
    * Walk through a directory's immediate contents without diving deeper.
    * A simple case of Files.walkFileTree provided by Java's NIO.2.
    * 
    * <p>WARNING: This method is best try only. All exceptions will be swallowed silently. For more specific behavior
    * you must use {@link Files#walkFileTree(Path, FileVisitor)} directly.</p>
    * 
    * @param directory directory to walk
    * @param basicFileVisitor callback to take action on visits
    */
   public static void walkFlat(final Path directory, final BasicPathVisitor basicFileVisitor)
   {
      walkDepth(directory, 1, basicFileVisitor);
   }
}
