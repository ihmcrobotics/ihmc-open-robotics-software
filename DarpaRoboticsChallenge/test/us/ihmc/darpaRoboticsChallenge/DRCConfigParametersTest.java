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
      assertFalse("Do not check in DRCConfigParameters.USE_SLIDER_FOR_POSE_PLAYBACK = true!!", DRCConfigParameters.USE_SLIDER_FOR_POSE_PLAYBACK);
      
      assertFalse("Do not check in DRCConfigParameters.USE_SUPER_DUPER_HIGH_RESOLUTION_FOR_COMMS = true!!", DRCConfigParameters.USE_SUPER_DUPER_HIGH_RESOLUTION_FOR_COMMS);
      
      assertFalse("Do not check in DRCConfigParameters.SEND_HIGH_SPEED_CONFIGURATION_DATA < 100!!", DRCConfigParameters.UI_JOINT_CONFIGURATION_UPDATE_MILLIS < 100);
      
      assertFalse("Do not check in MomentumBasedController.SPY_ON_MOMENTUM_BASED_CONTROLLER = true!!", MomentumBasedController.SPY_ON_MOMENTUM_BASED_CONTROLLER);

      assertTrue("Do not check in DRCConfigParameters.ALLOW_LAG_SIMULATION = false!!", DRCConfigParameters.ALLOW_LAG_SIMULATION);
      
      assertFalse("Do not check in DRCConfigParameters.ENABLE_LAG_SIMULATION_ON_START = true!!", DRCConfigParameters.ENABLE_LAG_SIMULATION_ON_START);
  
//      assertFalse("Do not check in DRCControllerDispatcher.RUN_SINGLE_THREADED = true!!", DRCControllerDispatcher.RUN_SINGLE_THREADED);
      
   }

}
