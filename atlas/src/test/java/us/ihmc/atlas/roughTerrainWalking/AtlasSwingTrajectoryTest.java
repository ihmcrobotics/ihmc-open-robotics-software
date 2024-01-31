package us.ihmc.atlas.roughTerrainWalking;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.HumanoidSwingTrajectoryTest;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;

/**
 * Some of these tests are disables because they exist for Nadia
 */
@Tag("humanoid-rough-terrain-slow")
public class AtlasSwingTrajectoryTest extends HumanoidSwingTrajectoryTest
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

   @Override
   @Disabled
   @Test
   public void testMultipleHeightFootsteps()
   {
      super.testMultipleHeightFootsteps();
   }

   @Override
   @Disabled
   @Test
   public void testNegativeSwingHeight()
   {
      super.testNegativeSwingHeight();
   }

   @Override
   @Disabled
   @Test
   public void testReallyHighFootstep()
   {
      super.testReallyHighFootstep();
   }

   @Override
   @Test
   public void testSelfCollisionAvoidance()
   {
      super.testSelfCollisionAvoidance();
   }
}
