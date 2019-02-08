package us.ihmc.atlas.pushRecovery;

import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.pushRecovery.DRCPushRecoveryTest;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasPushRecoveryTest extends DRCPushRecoveryTest
{
   @Override
   protected DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false)
      {
         @Override
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return new AtlasWalkingControllerParameters(RobotTarget.SCS, getJointMap(), getContactPointParameters())
            {
               @Override
               public boolean useOptimizationBasedICPController()
               {
                  return false;
               }
            };
         }
      };
   }

   @Override
   @Test
   public void testControllerFailureKicksIn() throws SimulationExceededMaximumTimeException
   {
      super.testControllerFailureKicksIn();
   }

   @Override
   @Test
   public void testLongBackwardPushWhileStanding() throws SimulationExceededMaximumTimeException
   {
      super.testLongBackwardPushWhileStanding();
   }

   @Override
   @Test
   public void testLongBackwardPushWhileStandingAfterControllerFailureKickedIn() throws SimulationExceededMaximumTimeException
   {
      super.testLongBackwardPushWhileStandingAfterControllerFailureKickedIn();
   }

   @Override
   @Test
   public void testLongForwardPushWhileStanding() throws SimulationExceededMaximumTimeException
   {
      super.testLongForwardPushWhileStanding();
   }

   @Override
   @Test
   public void testLongForwardPushWhileStandingAfterControllerFailureKickedIn() throws SimulationExceededMaximumTimeException
   {
      super.testLongForwardPushWhileStandingAfterControllerFailureKickedIn();
   }

   @Override
   @Test
   public void testPushICPOptimiWhileInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimiWhileInSwing();
   }

   @Override
   @Test
   public void testPushWhileInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushWhileInSwing();
   }

   @Override
   @Test
   public void testPushWhileInTransfer() throws SimulationExceededMaximumTimeException
   {
      super.testPushWhileInTransfer();
   }

   @Override
   @Test
   public void testPushWhileStanding() throws SimulationExceededMaximumTimeException
   {
      super.testPushWhileStanding();
   }

   @Override
   @Test
   public void testPushWhileStandingRecoveringAfterControllerFailureKickedIn() throws SimulationExceededMaximumTimeException
   {
      super.testPushWhileStandingRecoveringAfterControllerFailureKickedIn();
   }

   @Override
   @Test
   public void testRecoveringWithSwingSpeedUpWhileInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testRecoveringWithSwingSpeedUpWhileInSwing();
   }

   @Override
   @Test
   public void testRecoveryWhileInFlamingoStance() throws SimulationExceededMaximumTimeException
   {
      super.testRecoveryWhileInFlamingoStance();
   }
}
