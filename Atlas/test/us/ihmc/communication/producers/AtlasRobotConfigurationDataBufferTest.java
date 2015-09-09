package us.ihmc.communication.producers;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.tools.testing.TestPlanTarget;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.atlas.AtlasRobotVersion;

@DeployableTestClass(targets = TestPlanTarget.Flaky)
public class AtlasRobotConfigurationDataBufferTest extends RobotConfigurationDataBufferTest
{

   @Override
   public FullHumanoidRobotModel getFullRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, DRCRobotModel.RobotTarget.SCS, false).createFullRobotModel();
   }

}