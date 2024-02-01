package us.ihmc.tools.io;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.log.LogTools;

import java.nio.file.Path;

public class WorkspaceDirectoryTest
{
   @Test
   public void testWorkspaceDirectory()
   {
      WorkspaceDirectory workspaceDirectory = new WorkspaceDirectory("ihmc-open-robotics-software");
      LogTools.info("File access available: {}", workspaceDirectory.isFileAccessAvailable() ? "Yes" : "No");

      if (workspaceDirectory.isFileAccessAvailable())
      {
         Path directoryPath = workspaceDirectory.getFilesystemDirectory();
         LogTools.info("Directory path: {}", directoryPath);
         Assertions.assertNotNull(directoryPath, "Directory path is null");
      }
   }

   @Test
   public void testWorkspaceResourceDirectory()
   {
      WorkspaceResourceDirectory workspaceDirectory = new WorkspaceResourceDirectory(WorkspaceDirectoryTest.class);
      LogTools.info("File access available: {}", workspaceDirectory.isFileAccessAvailable() ? "Yes" : "No");

      if (workspaceDirectory.isFileAccessAvailable())
      {
         Path directoryPath = workspaceDirectory.getFilesystemDirectory();
         LogTools.info("Directory path: {}", directoryPath);
         Assertions.assertNotNull(directoryPath, "Directory path is null");
      }

      WorkspaceResourceDirectory directory;
      WorkspaceResourceFile file;

      directory = new WorkspaceResourceDirectory(HybridDirectoryTest.class);
      printDirectoryInfo(directory);
      Assertions.assertEquals("/us/ihmc/tools/io", directory.getPathNecessaryForClasspathLoading());
      Assertions.assertEquals("us.ihmc.tools.io", directory.getPathNecessaryForResourceExploring());
      file = new WorkspaceResourceFile(directory, "classLocatedResource.txt");
      printFileInfo(file);
      Assertions.assertNotNull(file.getClasspathResource());

      directory = new WorkspaceResourceDirectory(HybridDirectoryTest.class);
      printDirectoryInfo(directory);
      Assertions.assertEquals("/us/ihmc/tools/io", directory.getPathNecessaryForClasspathLoading());
      Assertions.assertEquals("us.ihmc.tools.io", directory.getPathNecessaryForResourceExploring());
      file = new WorkspaceResourceFile(directory, "testRootResource.txt");
      printFileInfo(file);
      Assertions.assertNull(file.getClasspathResource());

      directory = new WorkspaceResourceDirectory(HybridDirectoryTest.class, "/");
      printDirectoryInfo(directory);
      Assertions.assertEquals("/", directory.getPathNecessaryForClasspathLoading());
      Assertions.assertEquals("", directory.getPathNecessaryForResourceExploring());
      file = new WorkspaceResourceFile(directory, "testRootResource.txt");
      printFileInfo(file);
      Assertions.assertNotNull(file.getClasspathResource());

      directory = new WorkspaceResourceDirectory(HybridDirectoryTest.class, "subsequent");
      printDirectoryInfo(directory);
      Assertions.assertEquals("/us/ihmc/tools/io/subsequent", directory.getPathNecessaryForClasspathLoading());
      Assertions.assertEquals("us.ihmc.tools.io.subsequent", directory.getPathNecessaryForResourceExploring());
      file = new WorkspaceResourceFile(directory, "classSubsequentResource.txt");
      printFileInfo(file);
      Assertions.assertNotNull(file.getClasspathResource());

      directory = new WorkspaceResourceDirectory(HybridDirectoryTest.class, "subsequent/further");
      printDirectoryInfo(directory);
      Assertions.assertEquals("/us/ihmc/tools/io/subsequent/further", directory.getPathNecessaryForClasspathLoading());
      Assertions.assertEquals("us.ihmc.tools.io.subsequent.further", directory.getPathNecessaryForResourceExploring());
      file = new WorkspaceResourceFile(directory, "classFurtherSubsequentResource.txt");
      printFileInfo(file);
      Assertions.assertNotNull(file.getClasspathResource());
   }

   @Test
   public void testWorkspaceJavaDirectory()
   {
      WorkspaceJavaDirectory workspaceJavaDirectory;

      workspaceJavaDirectory = new WorkspaceJavaDirectory(WorkspaceJavaDirectory.class, "java");
      printDirectoryInfo(workspaceJavaDirectory);
   }

   private static void printFileInfo(WorkspaceResourceFile file)
   {
      LogTools.info("File path: {}", file.getFilesystemFile());
      LogTools.info("Path for resource loading: {}", file.getPathForResourceLoadingPathFiltered());
      LogTools.info("Classpath resource: {}", file.getClasspathResource());
   }

   private static void printDirectoryInfo(WorkspaceResourceDirectory directory)
   {
      LogTools.info("File access available: {}", directory.isFileAccessAvailable() ? "Yes" : "No");
      LogTools.info("Directory path: {}", directory.getFilesystemDirectory());
      LogTools.info("Class for loading: {}", directory.getClassForLoading());
      LogTools.info("Path necessary for classpath loading: {}", directory.getPathNecessaryForClasspathLoading());
      LogTools.info("Path necessary for resource exploring: {}", directory.getPathNecessaryForResourceExploring());

   }
   private static void printDirectoryInfo(WorkspaceJavaDirectory directory)
   {
      LogTools.info("File access available: {}", directory.isFileAccessAvailable() ? "Yes" : "No");
      LogTools.info("Directory path: {}", directory.getFilesystemDirectory());
   }
}
