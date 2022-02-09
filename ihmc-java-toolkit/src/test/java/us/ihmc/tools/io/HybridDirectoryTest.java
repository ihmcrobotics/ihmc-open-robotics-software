package us.ihmc.tools.io;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.resources.ResourceTools;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.TreeSet;

public class HybridDirectoryTest
{
   @Disabled // This doesn't really work on Bamboo and isn't designed to
   @Test
   public void testConstructors()
   {
      Path dotIHMC = Paths.get(System.getProperty("user.home")).resolve(".ihmc");
      String openRobotics = "ihmc-open-robotics-software";
      String subsequentPathToResourceFolder = "ihmc-java-toolkit/src/test/resources";
      HybridDirectory directory;
      HybridFile file;
      directory = new HybridDirectory(dotIHMC, openRobotics, subsequentPathToResourceFolder, "/");
      LogTools.info(directory.getWorkspaceDirectory());
      LogTools.info(directory.getExternalDirectory());
      LogTools.info(directory.getClassForLoading());
      LogTools.info(directory.getPathNecessaryForClasspathLoading());
      file = new HybridFile(directory, "testRootResource.txt");
      LogTools.info(file.getWorkspaceFile());
      LogTools.info(file.getExternalFile());
      LogTools.info(file.getClasspathResource());

      directory = new HybridDirectory(dotIHMC, openRobotics, subsequentPathToResourceFolder, getClass(), "/");
      LogTools.info(directory.getWorkspaceDirectory());
      LogTools.info(directory.getExternalDirectory());
      LogTools.info(directory.getClassForLoading());
      LogTools.info(directory.getPathNecessaryForClasspathLoading());
      file = new HybridFile(directory, "testRootResource.txt");
      LogTools.info(file.getWorkspaceFile());
      LogTools.info(file.getExternalFile());
      LogTools.info(file.getClasspathResource());

      directory = new HybridDirectory(dotIHMC, openRobotics, subsequentPathToResourceFolder, getClass());
      LogTools.info(directory.getWorkspaceDirectory());
      LogTools.info(directory.getExternalDirectory());
      LogTools.info(directory.getClassForLoading());
      LogTools.info(directory.getPathNecessaryForClasspathLoading());
      file = new HybridFile(directory, "classLocatedResource.txt");
      LogTools.info(file.getWorkspaceFile());
      LogTools.info(file.getExternalFile());
      LogTools.info(file.getClasspathResource());
      file = new HybridFile(directory, "subsequent/classSubsequentResource.txt");
      LogTools.info(file.getWorkspaceFile());
      LogTools.info(file.getExternalFile());
      LogTools.info(file.getClasspathResource());

      directory = new HybridDirectory(dotIHMC, openRobotics, subsequentPathToResourceFolder, getClass(), "/root");
      LogTools.info(directory.getWorkspaceDirectory());
      LogTools.info(directory.getExternalDirectory());
      LogTools.info(directory.getClassForLoading());
      LogTools.info(directory.getPathNecessaryForClasspathLoading());
      file = new HybridFile(directory, "relativeRootResource.txt");
      LogTools.info(file.getWorkspaceFile());
      LogTools.info(file.getExternalFile());
      LogTools.info(file.getClasspathResource());
      file = new HybridFile(directory, "subsequent/rootSubsequentResource.txt");
      LogTools.info(file.getWorkspaceFile());
      LogTools.info(file.getExternalFile());
      LogTools.info(file.getClasspathResource());

      directory = new HybridDirectory(dotIHMC, openRobotics, subsequentPathToResourceFolder, getClass(), "subsequent");
      LogTools.info(directory.getWorkspaceDirectory());
      LogTools.info(directory.getExternalDirectory());
      LogTools.info(directory.getClassForLoading());
      LogTools.info(directory.getPathNecessaryForClasspathLoading());
      file = new HybridFile(directory, "classSubsequentResource.txt");
      LogTools.info(file.getWorkspaceFile());
      LogTools.info(file.getExternalFile());
      LogTools.info(file.getClasspathResource());
      file = new HybridFile(directory, "purposefullyMisspelledResource.txt");
      LogTools.info(file.getWorkspaceFile());
      LogTools.info(file.getExternalFile());
      LogTools.info(file.getClasspathResource());
   }

   @Test
   public void testReadResourcesFromJar()
   {
      Path dotIHMC = Paths.get(System.getProperty("user.home")).resolve(".ihmc");
      String openRobotics = "ihmc-open-robotics-software";
      String subsequentPathToResourceFolder = "ihmc-java-toolkit/src/test/resources";
      HybridDirectory directory;
      directory = new HybridDirectory(dotIHMC, openRobotics, subsequentPathToResourceFolder, HybridDirectoryTest.class);

      for (String resource : ResourceTools.listResources(directory.getPathNecessaryForResourceExploring(), ".*"))
      {
         LogTools.info(resource);
      }

      LogTools.info("Walking flat");

      TreeSet<String> directoryNames = new TreeSet<>();
      TreeSet<String> fileNames = new TreeSet<>();
      directory.walkResourcesFlat((path, pathType) ->
      {
         LogTools.info("{}: {}", pathType.name(), path);
         if (pathType == BasicPathVisitor.PathType.FILE)
         {
            fileNames.add(path);
         }
         else
         {
            directoryNames.add(path);
         }
      });

      Assertions.assertTrue(fileNames.contains("testRootResource.txt"));
      Assertions.assertTrue(directoryNames.contains("us"));
      Assertions.assertTrue(directoryNames.contains("root"));

      directory = new HybridDirectory(dotIHMC, openRobotics, subsequentPathToResourceFolder, HybridDirectoryTest.class, "us/ihmc/tools/io");

      directoryNames.clear();
      fileNames.clear();
      directory.walkResourcesFlat((path, pathType) ->
      {
         LogTools.info("{}: {}", pathType.name(), path);
         if (pathType == BasicPathVisitor.PathType.FILE)
         {
            fileNames.add(path);
         }
         else
         {
            directoryNames.add(path);
         }
      });

      Assertions.assertTrue(fileNames.contains("classLocatedResource.txt"));
      Assertions.assertTrue(directoryNames.contains("files"));
      Assertions.assertTrue(directoryNames.contains("subsequent"));
   }
}
