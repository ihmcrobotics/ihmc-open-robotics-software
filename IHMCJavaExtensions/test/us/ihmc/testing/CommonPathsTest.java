package us.ihmc.testing;

import static org.junit.Assert.assertTrue;

import java.nio.file.Path;
import java.nio.file.Paths;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

/**
 * Package private so external classes don't see these internal tests.
 */
/* package-private */ class CommonPathsTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDeriveTestResourcePath()
   {
      Path path = Paths.get("testResources", "us/ihmc/testing/commonPathsTest");
      Path derivedPath = CommonPaths.deriveTestResourcesPath(this.getClass());

      assertTrue(path.compareTo(derivedPath) == 0);
   }
}
