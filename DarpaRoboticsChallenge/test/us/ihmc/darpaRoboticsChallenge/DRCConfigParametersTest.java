package us.ihmc.darpaRoboticsChallenge;

import static org.junit.Assert.assertFalse;

import org.junit.Test;

import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;

public class DRCConfigParametersTest
{

   @Test
   public void test()
   {
      assertFalse("Do not check in DRCConfigParameters.USE_SLIDER_FOR_POSE_PLAYBACK = true!!", DRCConfigParameters.USE_SLIDER_FOR_POSE_PLAYBACK);

      assertFalse("Do not check in DRCConfigParameters.USE_PERFECT_SENSORS = true!!", DRCConfigParameters.USE_PERFECT_SENSORS);
      
      assertFalse("Do not check in DRCConfigParameters.USE_SUPER_DUPER_HIGH_RESOLUTION_FOR_COMMS = true!!", DRCConfigParameters.USE_SUPER_DUPER_HIGH_RESOLUTION_FOR_COMMS);
      
      assertFalse("Do not check in DRCConfigParameters.SEND_HIGH_SPEED_CONFIGURATION_DATA = true!!", DRCConfigParameters.SEND_HIGH_SPEED_CONFIGURATION_DATA);
   
      assertFalse("Do not check in MomentumBasedController.SPY_ON_MOMENTUM_BASED_CONTROLLER = true!!", MomentumBasedController.SPY_ON_MOMENTUM_BASED_CONTROLLER);

      
   
   }

}
