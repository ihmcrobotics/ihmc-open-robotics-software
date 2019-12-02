package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

public class GoodFootstepPositionCheckerTest
{
   @Test
   public void testStanceFootPitchedTooMuch()
   {
      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();

      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(footPolygons);
      GoodFootstepPositionChecker checker = new GoodFootstepPositionChecker(parameters, snapper);
      parameters.setMaximumStepXWhenFullyPitched(0.3);
      parameters.setMinimumStepZWhenFullyPitched(0.05);


      PlanarRegionsListGenerator planarRegionGenerator = new PlanarRegionsListGenerator();
      planarRegionGenerator.rotate(new Quaternion(0.0, Math.toRadians(-30.0), 0.0));
      planarRegionGenerator.translate(0.0, 0.15, 0.0);
      planarRegionGenerator.addRectangle(0.3, 0.3);
      planarRegionGenerator.identity();
      planarRegionGenerator.translate(0.3, -0.15, -0.15);
      planarRegionGenerator.addRectangle(0.3, 0.3);

      PlanarRegionsList angledGround = planarRegionGenerator.getPlanarRegionsList();

      planarRegionGenerator = new PlanarRegionsListGenerator();
      planarRegionGenerator.translate(0.0, 0.15, 0.0);
      planarRegionGenerator.addRectangle(0.3, 0.3);
      planarRegionGenerator.identity();
      planarRegionGenerator.translate(0.3, -0.15, -0.15);
      planarRegionGenerator.addRectangle(0.3, 0.3);

      PlanarRegionsList flatGround = planarRegionGenerator.getPlanarRegionsList();

      snapper.setPlanarRegions(flatGround);
      checker.setPlanarRegions(flatGround);

      FootstepNode stanceNode = new FootstepNode(0.0, 0.15, 0.0, RobotSide.LEFT);
      FootstepNode childNode = new FootstepNode(0.3, -0.15, 0.0, RobotSide.RIGHT);

      boolean isValid = checker.isNodeValid(childNode, stanceNode);
      BipedalFootstepPlannerNodeRejectionReason rejectionReason = checker.getRejectionReason();
      assertTrue(isValid);
      assertEquals(null, rejectionReason);

      snapper.setPlanarRegions(angledGround);
      checker.setPlanarRegions(angledGround);

      isValid = checker.isNodeValid(childNode, stanceNode);
      assertFalse(isValid);
      assertEquals(BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_LOW_AND_FORWARD_WHEN_PITCHED, checker.getRejectionReason());

      // TODO check that this doesn't cause the other rejection reasons to fail if the pitch is flat.
   }
}
