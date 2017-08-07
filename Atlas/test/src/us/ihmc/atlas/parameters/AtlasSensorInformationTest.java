package us.ihmc.atlas.parameters;

import static org.junit.Assert.assertFalse;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class AtlasSensorInformationTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSendRobotDataToROSIsFalse()
   {
      assertFalse("Do not check in SEND_ROBOT_DATA_TO_ROS = true!!", AtlasSensorInformation.SEND_ROBOT_DATA_TO_ROS);
   }
}
