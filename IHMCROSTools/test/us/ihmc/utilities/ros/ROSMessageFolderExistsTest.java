package us.ihmc.utilities.ros;

import static org.junit.Assert.assertTrue;

import java.nio.file.Files;
import java.nio.file.Paths;

import org.junit.Test;

import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class ROSMessageFolderExistsTest
{
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testROSMessageFolderExists()
   {
      assertTrue(Files.exists(Paths.get(ROSMessageGenerator.messageFolder)));
   }
}
