package us.ihmc.valkyrie.pushRecovery;

import org.junit.Test;

import us.ihmc.avatar.DRCPushRecoveryWalkingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.SLOW, IntegrationCategory.VIDEO})
public class ValkyriePushRecoveryWalkingTest extends DRCPushRecoveryWalkingTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 26.7)
   @Test(timeout = 100000)
   public void testPushLeftEarlySwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushLeftEarlySwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 69.5)
   @Test(timeout = 100000)
   public void testPushLeftInitialTransferState() throws SimulationExceededMaximumTimeException
   {
      super.testPushLeftInitialTransferState();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 69.5)
   @Test(timeout = 100000)
   public void testPushRightInitialTransferState() throws SimulationExceededMaximumTimeException
   {
      super.testPushRightInitialTransferState();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 44.5)
   @Test(timeout = 100000)
   public void testPushRightLateSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushRightLateSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 44.5)
   @Test(timeout = 100000)
   public void testPushRightThenLeftMidSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushRightThenLeftMidSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 38.2)
   @Test(timeout = 100000)
   public void testPushRightTransferState() throws SimulationExceededMaximumTimeException
   {
      super.testPushRightTransferState();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 29.3, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   @Test(timeout = 100000)
   public void testPushTowardsTheBack() throws SimulationExceededMaximumTimeException
   {
      super.testPushTowardsTheBack();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 29.0)
   @Test(timeout = 100000)
   public void testPushTowardsTheFront() throws SimulationExceededMaximumTimeException
   {
      super.testPushTowardsTheFront();
   }
}