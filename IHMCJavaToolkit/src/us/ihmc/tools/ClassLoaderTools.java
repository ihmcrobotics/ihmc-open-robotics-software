package us.ihmc.tools;

import javax.management.IntrospectionException;

import java.io.*;
import java.lang.reflect.Method;
import java.net.URL;
import java.net.URLClassLoader;
import java.nio.file.FileSystem;
import java.nio.file.*;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.*;
import java.util.jar.Manifest;
import java.util.regex.Pattern;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;

public class ClassLoaderTools
{
   private static final boolean DEBUG = false;
   public static PathMatcher classMatcher = FileSystems.getDefault().getPathMatcher("glob:**/*.class");

   public static boolean isTopLevelDirectory(Path directory, Path... directories)
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
   
   public static List<Path> getTopLevelDirectories(Path... directories)
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

   public static void recursivelyGetResources(final ResourceHandler resourceHandler, String... directories) throws IOException
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

   private static void printIfDebug(String string)
   {
      if (DEBUG) System.out.println(string);
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

   public static void createZipBundle(OutputStream os, String... directories) throws IOException
   {
      createZipBundle(os, null, directories);
   }

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

   public static void main(String[] args) throws IOException
   {
      String[] directories = { "org", "org/xerial/snappy", "org/xerial/" };

      FileOutputStream os = new FileOutputStream(new File("/tmp/test.zip"));
      createZipBundle(os, directories);
      os.close();

      Path target = Files.createTempDirectory("test");
      copyToFileSystem(target, directories);
   }

   public interface ResourceHandler
   {
      public void handleResource(String resourcePath) throws IOException;
   }

   /*
    * Helper class from 
    * http://baptiste-wicht.com/posts/2010/05/tip-add-resources-dynamically-to-a-classloader.html
    * 
    * Source code exampels are free of rights
    */
   public static void addURLToSystemClassLoader(URL url) throws IntrospectionException
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
         throw new IntrospectionException("Error when adding url to system ClassLoader ");
      }
   }
}
