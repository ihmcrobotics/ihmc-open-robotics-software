package us.ihmc.tools.io;

import us.ihmc.commons.nio.PathTools;
import us.ihmc.log.LogTools;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

public class WorkspacePathTools
{
   /**
    * Use this method when applications are being run from source and need to access a project file.
    *
    * This method will find the directoryNameToAssumePresent and then traverse that path to find
    * the file system path to a resource.
    *
    * @param directoryNameToAssumePresent i.e. ihmc-open-robotics-software
    * @param subsequentPathToResourceFolder i.e. src/main/resources, or ihmc-java-toolkit/src/main/resources
    * @param resourcePathString i.e. us/ihmc/someResource.txt
    * @return absolute, normalized path to that directory, or null if fails
    */
   public static Path findPathToResource(String directoryNameToAssumePresent, String subsequentPathToResourceFolder, String resourcePathString)
   {
      return PathTools.findDirectoryInline(directoryNameToAssumePresent)
                      .resolve(subsequentPathToResourceFolder)
                      .resolve(resourcePathString)
                      .toAbsolutePath()
                      .normalize();
   }

   /**
    * Use this method when applications are being run from source and need to access a project file.
    *
    * This method searches for a directory in the path parts of the current working directory and
    * will also find it if it is a child of the current working directory.
    *
    * For example, if directoryNameToFind is ihmc-open-robotics-software and the current working directory is
    * ihmc-open-robotics-software/ihmc-java-toolkit/src, this will return the absolute path to ihmc-open-robotics-software.
    *
    * TODO: IHMC Commons's PathTools#findDirectoryInline is supposed to do this. Switch to that.
    *
    * @param directoryNameToFind
    * @return absolute path to that directory, or null if fails
    */
   public static Path handleWorkingDirectoryFuzziness(String directoryNameToFind)
   {
      Path absoluteWorkingDirectory = Paths.get(".").toAbsolutePath().normalize();
      Path pathBuiltFromSystemRoot = Paths.get("/").toAbsolutePath().normalize(); // start with system root

      boolean directoryFound = false;
      for (Path path : absoluteWorkingDirectory)
      {
         pathBuiltFromSystemRoot = pathBuiltFromSystemRoot.resolve(path); // building up the path

         if (path.toString().equals(directoryNameToFind))
         {
            directoryFound = true;
            break;
         }
      }

      if (!directoryFound && Files.exists(pathBuiltFromSystemRoot.resolve(directoryNameToFind))) // working directory is workspace
      {
         pathBuiltFromSystemRoot = pathBuiltFromSystemRoot.resolve(directoryNameToFind);
         directoryFound = true;
      }

      if (!directoryFound)
      {
         LogTools.warn("{} directory could not be found. Working directory: {} Search stopped at: {}",
                       directoryNameToFind,
                       absoluteWorkingDirectory,
                       pathBuiltFromSystemRoot);
         return null;
      }

      return pathBuiltFromSystemRoot;
   }
}
