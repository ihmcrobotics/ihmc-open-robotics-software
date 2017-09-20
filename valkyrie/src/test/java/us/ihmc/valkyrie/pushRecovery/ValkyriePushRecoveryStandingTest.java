package us.ihmc.valkyrie.pushRecovery;

import us.ihmc.avatar.DRCPushRecoveryStandingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

@ContinuousIntegrationPlan(categories = IntegrationCategory.IN_DEVELOPMENT)
public class ValkyriePushRecoveryStandingTest extends DRCPushRecoveryStandingTest
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
   
//   @Ignore
//   @QuarantinedTest("Need to fix the ICP planner so that after a push it does the right thing.")
//   @DeployableTestMethod(duration = 45.6)
//   @Test(timeout = 227903)
//   @Override
//   public void TestDoublePushForwardInDoubleSupportAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
//   {
//   }
   
   @Override
   public void testDoublePushForwardInDoubleSupportAndContinueWalking()
         throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testDoublePushForwardInDoubleSupportAndContinueWalking();
   }
   
   @Override
   public void testPushBackwardForwardInDoubleSupportAndContinueWalking()
         throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushBackwardForwardInDoubleSupportAndContinueWalking();
   }
   
   @Override
   public void testPushBackwardInDoubleSupportAndContinueWalking()
         throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushBackwardInDoubleSupportAndContinueWalking();
   }
   
   @Override
   public void testPushForwardInDoubleSupport() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushForwardInDoubleSupport();
   }
   
   @Override
   public void testPushForwardInDoubleSupportAndContinueWalking()
         throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushForwardInDoubleSupportAndContinueWalking();
   }

}
