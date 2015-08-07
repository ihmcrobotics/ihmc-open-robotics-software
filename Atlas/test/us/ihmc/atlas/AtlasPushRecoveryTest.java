package us.ihmc.atlas;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.pushRecovery.DRCPushRecoveryTest;
import us.ihmc.tools.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.tools.agileTesting.BambooPlanType;

@BambooPlan(planType = BambooPlanType.Slow)
public class AtlasPushRecoveryTest extends DRCPushRecoveryTest
{
   protected DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, AtlasRobotModel.AtlasTarget.SIM, false);
   }
}
