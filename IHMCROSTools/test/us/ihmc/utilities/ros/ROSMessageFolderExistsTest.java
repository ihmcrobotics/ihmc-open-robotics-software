package us.ihmc.utilities.ros;

import static org.junit.Assert.assertTrue;

import java.nio.file.Files;
import java.nio.file.Paths;

import org.junit.Test;

import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class ROSMessageFolderExistsTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testROSMessageFolderExists()
   {
//      assertTrue(Files.exists(Paths.get(ROSMessageFileCreator.messageRootFolder)));
   }
}
