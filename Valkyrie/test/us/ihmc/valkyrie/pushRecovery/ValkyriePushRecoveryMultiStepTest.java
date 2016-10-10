package us.ihmc.valkyrie.pushRecovery;

import us.ihmc.darpaRoboticsChallenge.DRCPushRecoveryMultiStepTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.tools.testing.TestPlanAnnotations.ContinuousIntegrationPlan;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.tools.testing.TestPlanTarget;

@ContinuousIntegrationPlan(targets = TestPlanTarget.InDevelopment)
public class ValkyriePushRecoveryMultiStepTest extends DRCPushRecoveryMultiStepTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(DRCRobotModel.RobotTarget.SCS, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }

   @Override
   protected void setForwardPushParameters()
   {
      forceMagnitude = 350.0;
      forceDuration = 0.2;
   }

   @Override
   protected void setBackwardPushParameters()
   {
      forceMagnitude = -400.0;
      forceDuration = 0.2;
   }
}
