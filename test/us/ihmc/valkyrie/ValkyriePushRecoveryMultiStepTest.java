package us.ihmc.valkyrie;

import us.ihmc.darpaRoboticsChallenge.DRCPushRecoveryMultiStepTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.tools.agileTesting.BambooPlanType;
import us.ihmc.tools.agileTesting.BambooAnnotations.BambooPlan;

@BambooPlan(planType = BambooPlanType.Slow)
public class ValkyriePushRecoveryMultiStepTest extends DRCPushRecoveryMultiStepTest
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
