package us.ihmc.atlas;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.pushRecovery.DRCPushRecoveryTest;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanTarget;

@DeployableTestClass(targets = TestPlanTarget.Slow)
public class AtlasPushRecoveryTest extends DRCPushRecoveryTest
{
   protected DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.SCS, false);
   }
}
