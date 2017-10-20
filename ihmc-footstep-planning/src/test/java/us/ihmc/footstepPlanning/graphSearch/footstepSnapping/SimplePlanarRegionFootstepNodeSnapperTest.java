package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionPolygonSnapperTest;
import us.ihmc.footstepPlanning.roughTerrainPlanning.FootstepPlannerOnRoughTerrainTest;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static junit.framework.TestCase.assertTrue;
import static org.junit.Assert.assertEquals;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = IntegrationCategory.IN_DEVELOPMENT)
public class SimplePlanarRegionFootstepNodeSnapperTest
{
   private final Random random = new Random(1209L);
   private final double epsilon = 1e-8;
   private final SideDependentList<ConvexPolygon2D> footPolygons = PlanningTestTools.createDefaultFootPolygons();

   @Test
   public void testIdentity()
   {
      SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(footPolygons);

      ConvexPolygon2D largeSquare = new ConvexPolygon2D();
      largeSquare.addVertex(100.0, 100.0);
      largeSquare.addVertex(-100.0, 100.0);
      largeSquare.addVertex(100.0, -100.0);
      largeSquare.addVertex(-100.0, -100.0);
      largeSquare.update();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(new PlanarRegion(new RigidBodyTransform(), largeSquare));
      snapper.setPlanarRegions(planarRegionsList);

      FootstepNode footstepNode = new FootstepNode(0.0, 0.0);
      FootstepNodeSnapData snapData = snapper.snapFootstepNode(footstepNode);
      assertTrue(snapData.getSnapTransform().epsilonEquals(new RigidBodyTransform(), epsilon));
      assertTrue(snapData.getCroppedFoothold().epsilonEquals(footPolygons.get(footstepNode.getRobotSide()), epsilon));
   }

   @Test
   public void testVerticalTranslation()
   {
      SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(footPolygons);

      ConvexPolygon2D largeSquare = new ConvexPolygon2D();
      largeSquare.addVertex(100.0, 100.0);
      largeSquare.addVertex(-100.0, 100.0);
      largeSquare.addVertex(100.0, -100.0);
      largeSquare.addVertex(-100.0, -100.0);
      largeSquare.update();

      RigidBodyTransform planarRegionToWorldTransform = new RigidBodyTransform();
      planarRegionToWorldTransform.setTranslationZ(1.0);

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(new PlanarRegion(planarRegionToWorldTransform, largeSquare));
      snapper.setPlanarRegions(planarRegionsList);

      FootstepNode footstepNode = new FootstepNode(0.0, 0.0);
      FootstepNodeSnapData snapData = snapper.snapFootstepNode(footstepNode);
      assertTrue(snapData.getSnapTransform().epsilonEquals(planarRegionToWorldTransform, epsilon));
      assertTrue(snapData.getCroppedFoothold().epsilonEquals(footPolygons.get(footstepNode.getRobotSide()), epsilon));
   }

   @Test
   public void testSimpleRotation()
   {
      SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(footPolygons);

      ConvexPolygon2D largeSquare = new ConvexPolygon2D();
      largeSquare.addVertex(100.0, 100.0);
      largeSquare.addVertex(-100.0, 100.0);
      largeSquare.addVertex(100.0, -100.0);
      largeSquare.addVertex(-100.0, -100.0);
      largeSquare.update();

      RigidBodyTransform planarRegionToWorldTransform = new RigidBodyTransform();
      planarRegionToWorldTransform.setRotationRoll(0.25 * Math.PI);

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(new PlanarRegion(planarRegionToWorldTransform, largeSquare));
      snapper.setPlanarRegions(planarRegionsList);

      FootstepNode footstepNode = new FootstepNode(0.0, 0.0);
      FootstepNodeSnapData snapData = snapper.snapFootstepNode(footstepNode);

      PlanarRegionPolygonSnapperTest.assertSurfaceNormalsMatchAndSnapPreservesXFromAbove(snapData.getSnapTransform(), planarRegionToWorldTransform);
   }

   @Test
   public void testForCompleteFootholds()
   {
      int numTests = 100;
      RigidBodyTransform nodeToWorldTransform = new RigidBodyTransform();

      ConvexPolygon2D largeSquare = new ConvexPolygon2D();
      largeSquare.addVertex(100.0, 100.0);
      largeSquare.addVertex(-100.0, 100.0);
      largeSquare.addVertex(100.0, -100.0);
      largeSquare.addVertex(-100.0, -100.0);
      largeSquare.update();

      for (int i = 0; i < numTests; i++)
      {
         FootstepNode node = FootstepNode.generateRandomFootstepNode(random, 5.0);
         FootstepNodeTools.getNodeTransform(node, nodeToWorldTransform);
         RigidBodyTransform snapTransform = generateRandomSnapTransform();

         RigidBodyTransform regionToWorld = new RigidBodyTransform(nodeToWorldTransform);
         regionToWorld.multiply(snapTransform);
         PlanarRegion planarRegion = new PlanarRegion(regionToWorld, largeSquare);

         SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(footPolygons);
         snapper.setPlanarRegions(new PlanarRegionsList(planarRegion));

         FootstepNodeSnapData snapData = snapper.snapFootstepNode(node);
         PlanarRegionPolygonSnapperTest.assertSurfaceNormalsMatchAndSnapPreservesXFromAbove(snapData.getSnapTransform(), regionToWorld);

         // TODO fix epsilon equals for ConvexPolygon2D, the points are the same but start in different quadrants
         // assertTrue(snapData.getCroppedFoothold().epsilonEquals(footPolygons.get(node.getRobotSide()), epsilon));

         assertEquals(snapData.getCroppedFoothold().getArea(), footPolygons.get(node.getRobotSide()).getArea(), epsilon);
      }
   }

   private RigidBodyTransform generateRandomSnapTransform()
   {
      double zTranslation = EuclidCoreRandomTools.generateRandomDouble(random, 1.0);
      double pitch = EuclidCoreRandomTools.generateRandomDouble(random, 0.25 * Math.PI);
      double roll = EuclidCoreRandomTools.generateRandomDouble(random, 0.25 * Math.PI);

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationYawPitchRoll(0.0, pitch, roll);
      transform.setTranslationZ(zTranslation);
      return transform;
   }
}
