package us.ihmc.avatar;

import static org.junit.Assert.assertFalse;

import org.junit.Test;

import us.ihmc.avatar.networkProcessor.modules.uiConnector.PacketsForwardedToTheUi;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class DRCConfigParametersTest
{
	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void test()
   {
      assertFalse("Do not check in PacketsForwardedToTheUi.SEND_HIGH_SPEED_CONFIGURATION_DATA < 100!!", PacketsForwardedToTheUi.UI_JOINT_CONFIGURATION_UPDATE_MILLIS < 100);
   }
}
