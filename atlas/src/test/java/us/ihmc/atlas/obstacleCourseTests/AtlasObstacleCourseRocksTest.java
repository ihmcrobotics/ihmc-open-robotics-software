package us.ihmc.atlas.obstacleCourseTests;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseRocksTest;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;

public class AtlasObstacleCourseRocksTest extends DRCObstacleCourseRocksTest
{

   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Tag("humanoid-obstacle")
   @Disabled
   @Test
   public void testWalkingOntoRocks()
   {
      super.testWalkingOntoRocks();
   }
}
