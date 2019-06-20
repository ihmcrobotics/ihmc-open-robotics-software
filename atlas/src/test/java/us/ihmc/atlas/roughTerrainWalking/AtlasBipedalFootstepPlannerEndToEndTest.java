package us.ihmc.atlas.roughTerrainWalking;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.AvatarBipedalFootstepPlannerEndToEndTest;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

@Disabled
public class AtlasBipedalFootstepPlannerEndToEndTest extends AvatarBipedalFootstepPlannerEndToEndTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      FootContactPoints<RobotSide> simulationContactPoints = new AdditionalSimulationContactPoints<>(RobotSide.values, 5, 3, true, false);
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false, simulationContactPoints);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   @Test
   public void testShortCinderBlockFieldWithPlanarRegionBipedalPlanner()
   {
      super.testShortCinderBlockFieldWithPlanarRegionBipedalPlanner();
   }

   @Override
   @Test
   public void testShortCinderBlockFieldWithAStar()
   {
      super.testShortCinderBlockFieldWithAStar();
   }

   @Override
   @Test
   public void testSteppingStonesWithAStar()
   {
      super.testSteppingStonesWithAStar();
   }

   @Override
   @Test
   public void testSteppingStonesWithPlanarRegionBipedalPlanner()
   {
      super.testSteppingStonesWithPlanarRegionBipedalPlanner();
   }
}
