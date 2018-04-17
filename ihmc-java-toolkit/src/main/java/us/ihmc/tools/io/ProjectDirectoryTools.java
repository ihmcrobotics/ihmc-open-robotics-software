package us.ihmc.tools.io;

import java.net.URI;
import java.net.URISyntaxException;
import java.net.URL;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

/**
 * Class with hacks to find the project folder of a running program
 * 
 * This only works if you run from source. The heuristics used are
 * 
 * - Your compiled class files are in the project folder
 * - Your current working directory is the project folder
 * - Your current working directory is the top level directory of your project folder
 * 
 * Requirements
 * - A build.gradle file is expected in the folder
 * - Standard maven structure is used
 *  
 * @author jesper
 *
 */
public class ProjectDirectoryTools
{
   private final static String buildFile = "build.gradle";
   private final static String sourceDirectory = "src";
   private final static String groupDirectory = "main";
   private final static String resourceDirectory = "resources";
         
   
   public static Path getProjectDirectory(Class<?> classInProject, String projectName)
   {
      // Step 1: Try super directories of the compiled class file
      URL myURL = classInProject.getResource(classInProject.getSimpleName() + ".class");
      try
      {
         URI myURI = myURL.toURI();
         String scheme = myURI.getScheme();
         if(scheme != null && scheme.equalsIgnoreCase("file"))
         {
            Path myPath = Paths.get(myURI);
            
            Path currentPath = myPath;
            while((currentPath = currentPath.getParent()) != null)
            {
               if(isProjectDirectory(projectName, currentPath))
               {
                  return currentPath;
               }
            }
            
         }
      }
      catch (URISyntaxException | IllegalArgumentException e)
      {
         // Ignore, try other heuristics
      }
      
      // Step 2: Try if working directory is current path
      Path workingDirectory = Paths.get(".").toAbsolutePath().normalize();
      if(workingDirectory.getFileName() != null)
      {  
         if(projectName.equals(workingDirectory.getFileName().toString()))
         {
            if(isProjectDirectory(projectName, workingDirectory))
            {
               return workingDirectory;
            }
         }
         
      }
      
      // Step 3: Try if the project is in the current working directory
      Path projectPath = Paths.get(projectName);
      if(isProjectDirectory(projectName, projectPath))
      {
         return projectPath;
      }
      
      return null;

   }
   
   /**
    * Try to get the main resource directory.
    * 
    *  This function is heuristic based. Use only to pre-populate a load/save dialog and don't automatically write
    * 
    * @param classInProject A class that is in the project you want the resource folder off
    * @param projectName Name of the project to match to
    * @return File with the resource directory or null if not known
    */
   public static Path getMainResourceDirectory(Class<?> classInProject, String projectName)
   {
      Path projectDirectory = getProjectDirectory(classInProject, projectName);
      if(projectDirectory != null)
      {
         Path resourceDirectoryPath = projectDirectory.resolve(Paths.get(sourceDirectory, groupDirectory, resourceDirectory));
         if(Files.exists(resourceDirectoryPath))
         {
            return resourceDirectoryPath;
         }
      }
      
      return null;
      
   }

   private static boolean isProjectDirectory(String projectName, Path currentPath)
   {
      Path file = currentPath.getFileName();
      if(file != null)
      {
         String name = currentPath.getFileName().toString();
         if(projectName.equals(name))
         {
            if(Files.exists(currentPath.resolve(buildFile)))
            {
               return true;
               
            }
         }
      }
      return false;
   }
   
   public static void main(String[] args)
   {
      System.out.println(getMainResourceDirectory(ProjectDirectoryTools.class, "ihmc-java-toolkit"));
   }
}
