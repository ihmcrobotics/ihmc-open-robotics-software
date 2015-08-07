package us.ihmc.valkyrie;

import us.ihmc.darpaRoboticsChallenge.DRCPushRecoveryStandingTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.tools.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.tools.agileTesting.BambooPlanType;

@BambooPlan(planType = BambooPlanType.InDevelopment)
public class ValkyriePushRecoveryStandingTest extends DRCPushRecoveryStandingTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(false, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }
   
//   @Ignore
//   @QuarantinedTest("Need to fix the ICP planner so that after a push it does the right thing.")
//   @EstimatedDuration(duration = 45.6)
//   @Test(timeout = 227903)
//   @Override
//   public void TestDoublePushForwardInDoubleSupportAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
//   {
//   }

}
