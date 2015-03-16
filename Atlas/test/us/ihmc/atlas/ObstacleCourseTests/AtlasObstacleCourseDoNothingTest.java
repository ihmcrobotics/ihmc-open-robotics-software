package us.ihmc.atlas.ObstacleCourseTests;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.obstacleCourseTests.DRCObstacleCourseDoNothingTest;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.utilities.code.agileTesting.BambooPlanType;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.BambooPlan;

@BambooPlan(planType = {BambooPlanType.Fast, BambooPlanType.VideoA})
public class AtlasObstacleCourseDoNothingTest extends DRCObstacleCourseDoNothingTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.DRC_NO_HANDS_UNPLUGGED_V4, AtlasRobotModel.AtlasTarget.SIM, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }
}
