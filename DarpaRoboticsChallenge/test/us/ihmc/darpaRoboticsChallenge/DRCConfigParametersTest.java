package us.ihmc.darpaRoboticsChallenge;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;

public class DRCConfigParametersTest
{

   @Test
   public void test()
   {
      assertFalse("Do not check in DRCConfigParameters.SEND_HIGH_SPEED_CONFIGURATION_DATA < 100!!", DRCConfigParameters.UI_JOINT_CONFIGURATION_UPDATE_MILLIS < 100);
      
      assertFalse("Do not check in MomentumBasedController.SPY_ON_MOMENTUM_BASED_CONTROLLER = true!!", MomentumBasedController.SPY_ON_MOMENTUM_BASED_CONTROLLER);      
      
      assertTrue("Do not add parameters to DRCConfigParameters.", DRCConfigParameters.class.getFields().length <= 14);

      assertFalse("Do not check in SEND_ROBOT_DATA_TO_ROS = true!!", DRCConfigParameters.SEND_ROBOT_DATA_TO_ROS);      
   }

}
