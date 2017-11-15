package us.ihmc.atlas.roughTerrainWalking;

import java.io.IOException;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.AvatarBipedalFootstepPlannerEndToEndTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = {IntegrationCategory.IN_DEVELOPMENT})
public class AtlasBipedalFootstepPlannerEndToEndTest extends AvatarBipedalFootstepPlannerEndToEndTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      FootContactPoints simulationContactPoints = new AdditionalSimulationContactPoints(5, 3, true, false);
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false, simulationContactPoints);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 120000)
   public void testShortCinderBlockFieldWithPlanarRegionBipedalPlanner() throws IOException
   {
      super.testShortCinderBlockFieldWithPlanarRegionBipedalPlanner();
   }

   @Override
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 120000)
   public void testShortCinderBlockFieldWithAStar() throws IOException
   {
      super.testShortCinderBlockFieldWithAStar();
   }

   @Override
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 120000)
   public void testSteppingStonesWithAStar() throws IOException
   {
      super.testSteppingStonesWithAStar();
   }

   @Override
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 120000)
   public void testSteppingStonesWithPlanarRegionBipedalPlanner() throws IOException
   {
      super.testSteppingStonesWithPlanarRegionBipedalPlanner();
   }
}
