package us.ihmc.valkyrie;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.pushRecovery.DRCPushRecoveryTest;
import us.ihmc.tools.agileTesting.BambooPlanType;
import us.ihmc.tools.agileTesting.BambooAnnotations.BambooPlan;

@BambooPlan(planType = BambooPlanType.InDevelopment)
public class ValkyriePushRecoveryTest extends DRCPushRecoveryTest
{
   protected DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(DRCRobotModel.RobotTarget.SCS, false);
   }
}
