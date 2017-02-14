package us.ihmc.utilities.ros;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class ROSMessageFolderExistsTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testROSMessageFolderExists()
   {
//      assertTrue(Files.exists(Paths.get(ROSMessageFileCreator.messageRootFolder)));
   }
}
