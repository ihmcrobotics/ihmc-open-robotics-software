package us.ihmc.atlas.pushRecovery;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.DRCPushRecoveryWalkingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationconstructionsettools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.SLOW, IntegrationCategory.VIDEO})
public class AtlasPushRecoveryWalkingTest extends DRCPushRecoveryWalkingTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.SCS, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 26.7)
   @Test(timeout = 130000)
   public void testPushLeftEarlySwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushLeftEarlySwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 30.2)
   @Test(timeout = 150000)
   public void testPushLeftInitialTransferState() throws SimulationExceededMaximumTimeException
   {
      super.testPushLeftInitialTransferState();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 44.5)
   @Test(timeout = 220000)
   public void testPushRightInitialTransferState() throws SimulationExceededMaximumTimeException
   {
      super.testPushRightInitialTransferState();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 44.5)
   @Test(timeout = 220000)
   public void testPushRightLateSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushRightLateSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 44.5)
   @Test(timeout = 220000)
   public void testPushRightThenLeftMidSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushRightThenLeftMidSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 31.2)
   @Test(timeout = 160000)
   public void testPushRightTransferState() throws SimulationExceededMaximumTimeException
   {
      super.testPushRightTransferState();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 29.3)
   @Test(timeout = 150000)
   public void testPushTowardsTheBack() throws SimulationExceededMaximumTimeException
   {
      super.testPushTowardsTheBack();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 29.0)
   @Test(timeout = 150000)
   public void testPushTowardsTheFront() throws SimulationExceededMaximumTimeException
   {
      super.testPushTowardsTheFront();
   }
}
