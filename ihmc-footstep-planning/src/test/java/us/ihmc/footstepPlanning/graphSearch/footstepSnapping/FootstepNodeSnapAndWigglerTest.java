package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class FootstepNodeSnapAndWigglerTest
{
   @Test
   public void testQPNotSolvedIfFootSufficientlyInside()
   {
      DefaultFootstepPlannerParameters footstepPlannerParameters = new DefaultFootstepPlannerParameters();
      footstepPlannerParameters.setMaximumXYWiggleDistance(0.1);
      footstepPlannerParameters.setMaximumYawWiggle(0.1);
      footstepPlannerParameters.setWiggleInsideDelta(0.05);

      double epsilon = 1e-5;
      double wiggleInsideDelta = footstepPlannerParameters.getWiggleInsideDelta();
      double pX = 0.5 * PlannerTools.footLength;
      double pY = 0.5 * PlannerTools.footWidth;

      ConvexPolygon2D largeEnoughPolygon = new ConvexPolygon2D();
      largeEnoughPolygon.addVertex(pX + (wiggleInsideDelta + epsilon), pY + (wiggleInsideDelta + epsilon));
      largeEnoughPolygon.addVertex(pX + (wiggleInsideDelta + epsilon), -pY - (wiggleInsideDelta + epsilon));
      largeEnoughPolygon.addVertex(-pX - (wiggleInsideDelta + epsilon), pY + (wiggleInsideDelta + epsilon));
      largeEnoughPolygon.addVertex(-pX - (wiggleInsideDelta + epsilon), -pY - (wiggleInsideDelta + epsilon));
      largeEnoughPolygon.update();

      ConvexPolygon2D tooSmallPolygon = new ConvexPolygon2D();
      tooSmallPolygon.addVertex(pX + (wiggleInsideDelta - epsilon), pY + (wiggleInsideDelta - epsilon));
      tooSmallPolygon.addVertex(pX + (wiggleInsideDelta - epsilon), -pY - (wiggleInsideDelta - epsilon));
      tooSmallPolygon.addVertex(-pX - (wiggleInsideDelta - epsilon), pY + (wiggleInsideDelta - epsilon));
      tooSmallPolygon.addVertex(-pX - (wiggleInsideDelta - epsilon), -pY - (wiggleInsideDelta - epsilon));
      tooSmallPolygon.update();

//      SideDependentList<ConvexPolygon2D> defaultFootPolygons = PlannerTools.createDefaultFootPolygons();
//      FootstepNodeSnapAndWiggleTester snapAndWiggler = new FootstepNodeSnapAndWiggleTester(defaultFootPolygons, footstepPlannerParameters);
//
//      // test region meeting wiggleInsideDelta requirement doesn't call wiggle method
//      FootstepNode footstepNode = new FootstepNode(0.0, 0.0, 0.0, RobotSide.LEFT);
//      snapAndWiggler.setPlanarRegions(new PlanarRegionsList(new PlanarRegion(new RigidBodyTransform(), largeEnoughPolygon)));
//      snapAndWiggler.snapFootstepNode(footstepNode);
//      Assertions.assertFalse(snapAndWiggler.dirtyBit);
//
//      // test region not meeting wiggleInsideDelta requirement calls wiggle method
//      snapAndWiggler.snapFootstepNode(footstepNode);
//      snapAndWiggler.setPlanarRegions(new PlanarRegionsList(new PlanarRegion(new RigidBodyTransform(), tooSmallPolygon)));
//      snapAndWiggler.snapFootstepNode(footstepNode);
//      Assertions.assertTrue(snapAndWiggler.dirtyBit);
   }

//   private class FootstepNodeSnapAndWiggleTester extends FootstepNodeSnapAndWiggler
//   {
//      boolean dirtyBit = false;
//
//      public FootstepNodeSnapAndWiggleTester(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame, FootstepPlannerParametersReadOnly parameters)
//      {
//         super(footPolygonsInSoleFrame, parameters);
//      }
//
//      @Override
//      RigidBodyTransform getWiggleTransformInPlanarRegionFrame(ConvexPolygon2D footholdPolygon)
//      {
//         dirtyBit = true;
//         return super.getWiggleTransformInPlanarRegionFrame(footholdPolygon);
//      }
//   }

   @Test
   public void testSimpleSnap()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(0.0, 0.0, -0.2);
      generator.addRectangle(5.0, 5.0);
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      SideDependentList<ConvexPolygon2D> defaultFootPolygons = PlannerTools.createDefaultFootPolygons();
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      FootstepNodeSnapAndWiggler snapAndWiggler = new FootstepNodeSnapAndWiggler(defaultFootPolygons, parameters);
      snapAndWiggler.setPlanarRegions(planarRegionsList);

      FootstepNode footstepNode = new FootstepNode(0.1, 0.2, 1.0, RobotSide.RIGHT);
      FootstepNodeSnapData snapData = snapAndWiggler.snapFootstepNode(footstepNode);
      System.out.println(snapData.getSnapTransform());
      System.out.println();
      System.out.println(snapData.getSnappedNodeTransform(footstepNode));
   }
}
