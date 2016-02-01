package us.ihmc.darpaRoboticsChallenge;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.modules.uiConnector.PacketsForwardedToTheUi;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class DRCConfigParametersTest
{

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void test()
   {
      assertFalse("Do not check in PacketsForwardedToTheUi.SEND_HIGH_SPEED_CONFIGURATION_DATA < 100!!", PacketsForwardedToTheUi.UI_JOINT_CONFIGURATION_UPDATE_MILLIS < 100);
      
      assertFalse("Do not check in MomentumBasedController.SPY_ON_MOMENTUM_BASED_CONTROLLER = true!!", MomentumBasedController.SPY_ON_MOMENTUM_BASED_CONTROLLER);      
      
      assertTrue("Do not add parameters to DRCConfigParameters.", DRCConfigParameters.class.getFields().length <= 8);

      assertFalse("Do not check in SEND_ROBOT_DATA_TO_ROS = true!!", DRCConfigParameters.SEND_ROBOT_DATA_TO_ROS);      
   }

}
