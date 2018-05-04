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

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST, IntegrationCategory.VIDEO})
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

   @ContinuousIntegrationTest(estimatedDuration = 36.8)
   @Test(timeout = 180000)
   public void testPushLeftEarlySwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushLeftEarlySwing(700.0);
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 70.4)
   @Test(timeout = 350000)
   public void testPushLeftInitialTransferState() throws SimulationExceededMaximumTimeException
   {
      super.testPushLeftInitialTransferState();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 62.6)
   @Test(timeout = 310000)
   public void testPushRightInitialTransferState() throws SimulationExceededMaximumTimeException
   {
      super.testPushRightInitialTransferState();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 38.1)
   @Test(timeout = 190000)
   public void testPushRightLateSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushRightLateSwing();
   }

   @ContinuousIntegrationTest(estimatedDuration = 63.1)
   @Test(timeout = 320000)
   public void testPushRightThenLeftMidSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushRightThenLeftMidSwing(700.0);
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 37.9)
   @Test(timeout = 190000)
   public void testPushRightTransferState() throws SimulationExceededMaximumTimeException
   {
      super.testPushRightTransferState();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 29.3, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   @Test(timeout = 200000)
   public void testPushTowardsTheBack() throws SimulationExceededMaximumTimeException
   {
      super.testPushTowardsTheBack();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 37.7)
   @Test(timeout = 190000)
   public void testPushTowardsTheFront() throws SimulationExceededMaximumTimeException
   {
      super.testPushTowardsTheFront();
   }
}