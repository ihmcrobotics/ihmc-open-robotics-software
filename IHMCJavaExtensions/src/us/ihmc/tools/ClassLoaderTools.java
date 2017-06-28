package us.ihmc.tools;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.lang.reflect.Method;
import java.net.URL;
import java.net.URLClassLoader;
import java.nio.file.FileSystem;
import java.nio.file.FileSystems;
import java.nio.file.FileVisitResult;
import java.nio.file.Files;
import java.nio.file.InvalidPathException;
import java.nio.file.Path;
import java.nio.file.PathMatcher;
import java.nio.file.Paths;
import java.nio.file.SimpleFileVisitor;
import java.nio.file.StandardCopyOption;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Enumeration;
import java.util.HashSet;
import java.util.List;
import java.util.jar.Manifest;
import java.util.regex.Pattern;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;

/**
 * Utility class to do awful things with class loaders.
 * 
 * Specifically, add support to extract resources from the current class path and copy them to the file system or zip them up.
 * 
 * In general, this class only works for classpaths pointing to the file system or .jar files
 * 
 * @author Jesper Smith
 * 
 */
public class ClassLoaderTools
{


   private static final boolean DEBUG = false;

   private static PathMatcher classMatcher = FileSystems.getDefault().getPathMatcher("glob:**/*.class");
   

   /**
    * Copies all files on the classpath in the given directories (packages) to the Path target on the file system 
    * 
    * Works only for classpaths pointing to the file system or .jar files
    * 
    * @param target Target to copy to on the file system
    * @param directories Directories (packages) to copy on the classPath
    * 
    * @throws IOException If the files cannot be written to disk
    */
   public static void copyToFileSystem(final Path target, String... directories) throws IOException
   {
      recursivelyGetResources(new ResourceHandler()
      {

         @Override
         public void handleResource(String resourcePath) throws IOException
         {
            InputStream resource = ClassLoaderTools.class.getClassLoader().getResourceAsStream(resourcePath);
            Path destination = target.resolve(resourcePath);
            Files.createDirectories(destination.getParent());
            Files.copy(resource, destination, StandardCopyOption.REPLACE_EXISTING);
            resource.close();
         }

      }, directories);

   }

   /**
    * Copies all files on the classpath in the given directories (packages) to the outputstream in zip format
    * 
    * Works only for classpaths pointing to the file system or .jar files
    * 
    * @param os Outputstream to write zip file to
    * @param exclude Exclude files that match the given pattern
    * @param directories Directories (packages) to copy on the classPath
    * @throws IOException If the files cannot be written to the zip file.
    */
   public static void createZipBundle(OutputStream os, final Pattern exclude, String... directories) throws IOException
   {
      final ZipOutputStream stream = new ZipOutputStream(os);

      recursivelyGetResources(new ResourceHandler()
      {
         private final HashSet<String> names = new HashSet<>();

         @Override
         public void handleResource(String resourcePath) throws IOException
         {
            if (exclude != null)
            {
               if (exclude.matcher(resourcePath).matches())
               {
                  return;
               }
            }
            ZipEntry entry = new ZipEntry(resourcePath);

            if (names.add(entry.getName()))
            {
               InputStream resource = ClassLoaderTools.class.getClassLoader().getResourceAsStream(resourcePath);
               stream.putNextEntry(entry);
               copyStream(resource, stream);
               stream.closeEntry();

               resource.close();
            }
         }
      }, directories);

      stream.close();
   }

   /**
    * Copies all files on the classpath in the given directories (packages) to the outputstream in zip format
    * 
    * Works only for classpaths pointing to the file system or .jar files
    * 
    * @param os Outputstream to write zip file to
    * @param directories Directories (packages) to copy on the classPath
    * @throws IOException If the files cannot be written to the zip file.
    */
   public static void createZipBundle(OutputStream os, String... directories) throws IOException
   {
      createZipBundle(os, null, directories);
   }
   
   /**
    * Awful hack to add URL's to the system class loader, useful for adding resources that get loaded deep inside libraries. 
    * 
    * Helper class from 
    * http://baptiste-wicht.com/posts/2010/05/tip-add-resources-dynamically-to-a-classloader.html
    * 
    * Original source is licensed CC BY 4.0
    */
   public static void addURLToSystemClassLoader(URL url) throws IOException
   {
      URLClassLoader systemClassLoader = (URLClassLoader) ClassLoader.getSystemClassLoader();
      Class<URLClassLoader> classLoaderClass = URLClassLoader.class;

      try
      {
         Method method = classLoaderClass.getDeclaredMethod("addURL", new Class[] { URL.class });
         method.setAccessible(true);
         method.invoke(systemClassLoader, new Object[] { url });
      }
      catch (Throwable t)
      {
         t.printStackTrace();
         throw new IOException("Error when adding url to system ClassLoader ");
      }
   }

   private static List<Path> getTopLevelDirectories(Path... directories)
   {
      ArrayList<Path> topLevelDirectories = new ArrayList<>();
      for (Path directory : directories)
      {
         if (isTopLevelDirectory(directory, directories))
         {
            topLevelDirectories.add(directory);
         }
      }
      return topLevelDirectories;
   }

   private static boolean isTopLevelDirectory(Path directory, Path... directories)
   {
      if (Files.exists(directory))
      {
         for (Path testDirectory : directories)
         {
            if (!directory.equals(testDirectory) && directory.startsWith(testDirectory))
            {
               return false;
            }
         }
      }
      else
      {
         return false;
      }

      return true;
   }


   private static void printIfDebug(String string)
   {
      if (DEBUG) System.out.println(string);
   }

   private static void recursivelyGetResources(final ResourceHandler resourceHandler, String... directories) throws IOException
   {
      HashSet<String> classPathSet = new HashSet<String>();
      
      // Everything is awful
      Enumeration<URL> manifests = ClassLoaderTools.class.getClassLoader().getResources("META-INF/MANIFEST.MF");
      while(manifests.hasMoreElements())
      {
         URL next = manifests.nextElement();
         InputStream stream = next.openStream();
         Manifest manifest = new Manifest(stream);
         String jarClassPath = manifest.getMainAttributes().getValue("Class-Path");
         if(jarClassPath != null)
         {
            classPathSet.addAll(Arrays.asList(jarClassPath.split(" ")));
         }
         stream.close();
      }
      
      classPathSet.addAll(Arrays.asList(System.getProperty("java.class.path").split(Pattern.quote(File.pathSeparator))));
     
      for(String nextToken : classPathSet)
      {
         Path classPath = null;
         try
         {
            classPath = Paths.get(nextToken);
         }
         catch(InvalidPathException e)
         {
            printIfDebug("Couldn't find " + nextToken);
            continue;
         }
         
         // Some jars define non-existing classes on their Class-Path. Just silently ignore those
         if(!Files.exists(classPath))
         {
            continue;
         }

         final Path path;
         FileSystem fs;
         if (!Files.isDirectory(classPath))
         {
            try
            {
               fs = FileSystems.newFileSystem(classPath, null);
            } catch (Exception e)
            {
               if(DEBUG)
               {
                  printIfDebug("Problem creating a FileSystem for the following classpath entry, skipping: " + nextToken);
                  StringWriter writer = new StringWriter();
                  e.printStackTrace(new PrintWriter(writer));
                  printIfDebug("Showing stack trace string:\n" + writer.toString());
               }
               continue;
            }
            path = fs.getPath("/");
         }
         else
         {
            fs = null;
            path = classPath;
         }

         Path[] absoluteDirectories = new Path[directories.length];
         for (int i = 0; i < directories.length; i++)
         {
            absoluteDirectories[i] = path.resolve(directories[i]);
         }

         for (Path subdir : getTopLevelDirectories(absoluteDirectories))
         {
            if (Files.exists(subdir))
            {
               Files.walkFileTree(subdir, new SimpleFileVisitor<Path>()
               {
                  @Override
                  public FileVisitResult visitFile(Path file, BasicFileAttributes attrs) throws IOException
                  {
                     if (!classMatcher.matches(file))
                     {
                        resourceHandler.handleResource(path.relativize(file).toString());
                     }
                     return FileVisitResult.CONTINUE;
                  }
               });
            }
         }

         if (fs != null)
         {
            fs.close();
         }
      }
   }
   
   private static void copyStream(InputStream is, OutputStream os) throws IOException
   {
      byte[] buffer = new byte[8192];
      int n;
      while ((n = is.read(buffer)) > 0)
      {
         os.write(buffer, 0, n);
      }
   }



   private interface ResourceHandler
   {
      public void handleResource(String resourcePath) throws IOException;
   }


}
