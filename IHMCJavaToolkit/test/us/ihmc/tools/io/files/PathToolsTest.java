package us.ihmc.tools.io.files;

import static org.junit.Assert.assertTrue;

import java.nio.file.FileSystems;
import java.nio.file.FileVisitResult;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;

import org.apache.commons.lang3.StringUtils;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.commons.CommonPaths;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.io.printing.PrintTools;

public class PathToolsTest
{
   private static final Path TEST_RESOURCES_PATH = CommonPaths.deriveTestResourcesPath(PathToolsTest.class);
   private static final Path[] TEST_DIRECTORIES = {TEST_RESOURCES_PATH.resolve("testDir1"), TEST_RESOURCES_PATH.resolve("testDir2")};
   private static final Path[] TEST_FILES = {TEST_RESOURCES_PATH.resolve("testFile1.txt"), TEST_RESOURCES_PATH.resolve("testFile2.txt")};
   
   @Before
   public void setUp()
   {
      FileTools.ensureDirectoryExists(TEST_RESOURCES_PATH);
   }
   
	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testFindAllPathsRecursivelyThatMatchRegex()
   {
      List<Path> matchingPaths = PathTools.findAllPathsRecursivelyThatMatchRegex(Paths.get("testResources"), ".*[\\\\/]PathTools\\.java\\.fake$");
      
      PrintTools.info(this, "Matched " + matchingPaths.size() + " file(s).");
      
      assertTrue("Didn't match exactly one file.", matchingPaths.size() == 1);
   }
   
	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetBaseName()
   {
      Path pathToThisTest = FileSystems.getDefault().getPath("test/us/ihmc/utilities/io/files/PathToolsTest.java");
      
      String baseName = PathTools.getBaseName(pathToThisTest);
      
      PrintTools.info(this, "Base name of this test: " + baseName);
      
      assertTrue("Base name not correct.", baseName.equals(PathToolsTest.class.getSimpleName()));
   }
   
	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetExtension()
   {
      Path pathToThisTest = FileSystems.getDefault().getPath("test/us/ihmc/utilities/io/files/PathToolsTest.java");
      
      String extensionName = PathTools.getExtension(pathToThisTest);
      
      PrintTools.info(this, "Extension name of this test: " + extensionName);
      
      assertTrue("Extension name not correct.", extensionName.equals("java"));
   }
   
	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testFirstPathMatchingGlob()
   {
      String camelCasedClassSimpleName = StringUtils.uncapitalize(PathToolsTest.class.getSimpleName());
      
      PrintTools.info(this, "Camel cased simple name: " + camelCasedClassSimpleName);
      
      Path firstPath = PathTools.findFirstPathMatchingGlob(Paths.get("testResources"), "**/" + camelCasedClassSimpleName);
      
      if (firstPath == null)
         PrintTools.error(this, "Path not found!");
      
      assertTrue("directoryHasGlob not working.", PathTools.directoryHasGlob(Paths.get("testResources"), "**/" + camelCasedClassSimpleName));
      
      PrintTools.info(this, "First path: " + firstPath.toString());
      PrintTools.info(this, "First path fileName: " + firstPath.getFileName());
      
      assertTrue("First path not correct.", firstPath.getFileName().toString().equals(camelCasedClassSimpleName));
   }
	
	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testWalkTreeFlat()
   {
      PathTools.walkFlat(TEST_RESOURCES_PATH, new BasicPathVisitor()
      {
         int resultCount = 0;
         
         @Override
         public FileVisitResult visitPath(Path path, PathType pathType)
         {
            PrintTools.info(path.toString() + " " + pathType.toString());            
            if (path.equals(TEST_DIRECTORIES[0]) || path.equals(TEST_DIRECTORIES[1]))
            {
               assertTrue("Falsely reported directory", pathType.equals(PathType.DIRECTORY));
            }
            else if (path.equals(TEST_FILES[0]) || path.equals(TEST_FILES[1]))
            {
               assertTrue("Falsely reported file", pathType.equals(PathType.FILE));
            }
            
            resultCount++;
            
            assertTrue("Parent was included", resultCount <= 4);
            
            return FileVisitResult.CONTINUE;
         }
      });
   }
}
