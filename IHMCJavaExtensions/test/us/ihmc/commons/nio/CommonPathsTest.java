package us.ihmc.commons.nio;

import static org.junit.Assert.assertTrue;

import java.nio.file.Path;
import java.nio.file.Paths;

import org.junit.Test;

import us.ihmc.commons.nio.CommonPaths;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class CommonPathsTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDeriveTestResourcePath()
   {
      Path path = Paths.get("testResources", "us", "ihmc", "commons", "nio", "commonPathsTest");
      Path derivedPath = CommonPaths.deriveTestResourcesPath(this.getClass());

      assertTrue(path.compareTo(derivedPath) == 0);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDeriveResourcePath()
   {
      Path path = Paths.get("resources", "us", "ihmc", "commons", "nio", "commonPaths");
      Path derivedPath = CommonPaths.deriveResourcesPath(CommonPaths.class);

      assertTrue(path.compareTo(derivedPath) == 0);
   }
}
