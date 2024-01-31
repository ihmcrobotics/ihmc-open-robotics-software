package us.ihmc.atlas.obstacleCourseTests;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseEveryBuildTest;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;

@Tag("humanoid-flat-ground")
public class AtlasObstacleCourseEveryBuildTest extends DRCObstacleCourseEveryBuildTest
{
   private final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }
   
   @Override
   @Test
   public void testSimpleFlatGroundScript()
   {
      super.testSimpleFlatGroundScript();
   }
   
   @Override
   @Test
   public void testWalkingUpToRampWithLongSteps()
   {
      super.testWalkingUpToRampWithLongSteps();
   }
}
