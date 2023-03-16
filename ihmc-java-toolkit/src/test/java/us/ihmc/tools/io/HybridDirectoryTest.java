package us.ihmc.tools.io;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.resources.ResourceTools;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.TreeSet;

public class HybridDirectoryTest
{
   @Test
   public void testConstructors()
   {
      Path dotIHMC = Paths.get(System.getProperty("user.home")).resolve(".ihmc");
      HybridResourceDirectory directory;
      HybridResourceFile file;

      directory = new HybridResourceDirectory(dotIHMC, HybridDirectoryTest.class);
      printDirectoryInfo(directory);
      Assertions.assertEquals("us/ihmc/tools/io", directory.getPathNecessaryForClasspathLoading());
      Assertions.assertEquals("us.ihmc.tools.io", directory.getPathNecessaryForResourceExploring());
      file = new HybridResourceFile(directory, "testRootResource.txt");
      printFileInfo(file);

      directory = new HybridResourceDirectory(dotIHMC, HybridDirectoryTest.class, "/");
      printDirectoryInfo(directory);
      Assertions.assertEquals("/", directory.getPathNecessaryForClasspathLoading());
      Assertions.assertEquals("", directory.getPathNecessaryForResourceExploring());
      file = new HybridResourceFile(directory, "testRootResource.txt");
      printFileInfo(file);

      directory = new HybridResourceDirectory(dotIHMC, HybridDirectoryTest.class, "stuff");
      printDirectoryInfo(directory);
      Assertions.assertEquals("us/ihmc/tools/io/stuff", directory.getPathNecessaryForClasspathLoading());
      Assertions.assertEquals("us.ihmc.tools.io.stuff", directory.getPathNecessaryForResourceExploring());
      file = new HybridResourceFile(directory, "testRootResource.txt");
      printFileInfo(file);

      directory = new HybridResourceDirectory(dotIHMC, HybridDirectoryTest.class, "stuff/moreStuff");
      printDirectoryInfo(directory);
      Assertions.assertEquals("us/ihmc/tools/io/stuff/moreStuff", directory.getPathNecessaryForClasspathLoading());
      Assertions.assertEquals("us.ihmc.tools.io.stuff.moreStuff", directory.getPathNecessaryForResourceExploring());
      file = new HybridResourceFile(directory, "testRootResource.txt");
      printFileInfo(file);
   }

   private static void printFileInfo(HybridResourceFile file)
   {
      LogTools.info("Workspace file: {}", file.getWorkspaceFile());
      LogTools.info("External file: {}", file.getExternalFile());
      LogTools.info("Classpath resource: {}", file.getClasspathResource());
   }

   private static void printDirectoryInfo(HybridResourceDirectory directory)
   {
      LogTools.info("Workspace directory: {}", directory.getWorkspaceDirectory());
      LogTools.info("External directory: {}", directory.getExternalDirectory());
      LogTools.info("Class for loading: {}", directory.getClassForLoading());
      LogTools.info("Path necessary for classpath loading: {}", directory.getPathNecessaryForClasspathLoading());
      LogTools.info("Path necessary for resource exploring: {}", directory.getPathNecessaryForResourceExploring());
   }

   //   @Test
//   public void testReadResourcesFromJar()
//   {
//      Path dotIHMC = Paths.get(System.getProperty("user.home")).resolve(".ihmc");
//      String openRobotics = "ihmc-open-robotics-software";
//      String subsequentPathToResourceFolder = "ihmc-java-toolkit/src/test/resources";
//      HybridDirectory directory;
//      directory = new HybridDirectory(dotIHMC, openRobotics, subsequentPathToResourceFolder, HybridDirectoryTest.class);
//
//      for (String resource : ResourceTools.listResources(directory.getPathNecessaryForResourceExploring(), ".*"))
//      {
//         LogTools.info(resource);
//      }
//
//      LogTools.info("Walking flat");
//
//      TreeSet<String> directoryNames = new TreeSet<>();
//      TreeSet<String> fileNames = new TreeSet<>();
//      directory.walkResourcesFlat((path, pathType) ->
//      {
//         LogTools.info("{}: {}", pathType.name(), path);
//         if (pathType == BasicPathVisitor.PathType.FILE)
//         {
//            fileNames.add(path);
//         }
//         else
//         {
//            directoryNames.add(path);
//         }
//      });
//
//      Assertions.assertTrue(fileNames.contains("testRootResource.txt"));
//      Assertions.assertTrue(directoryNames.contains("us"));
//      Assertions.assertTrue(directoryNames.contains("root"));
//
//      directory = new HybridDirectory(dotIHMC, openRobotics, subsequentPathToResourceFolder, HybridDirectoryTest.class, "us/ihmc/tools/io");
//
//      directoryNames.clear();
//      fileNames.clear();
//      directory.walkResourcesFlat((path, pathType) ->
//      {
//         LogTools.info("{}: {}", pathType.name(), path);
//         if (pathType == BasicPathVisitor.PathType.FILE)
//         {
//            fileNames.add(path);
//         }
//         else
//         {
//            directoryNames.add(path);
//         }
//      });
//
//      Assertions.assertTrue(fileNames.contains("classLocatedResource.txt"));
//      Assertions.assertTrue(directoryNames.contains("files"));
//      Assertions.assertTrue(directoryNames.contains("subsequent"));
//   }
}
