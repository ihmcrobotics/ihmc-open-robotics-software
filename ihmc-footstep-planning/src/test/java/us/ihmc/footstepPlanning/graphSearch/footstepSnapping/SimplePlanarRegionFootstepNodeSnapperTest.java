package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionPolygonSnapperTest;
import us.ihmc.footstepPlanning.polygonSnapping.PolygonSnapperVisualizer;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.commons.thread.ThreadTools;

import java.util.Random;

import static us.ihmc.robotics.Assert.*;

public class SimplePlanarRegionFootstepNodeSnapperTest
{
   private final Random random = new Random(1209L);
   private final double epsilon = 1e-8;
   private final SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
   private final SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(footPolygons);
   private final ConvexPolygon2D unitSquare = new ConvexPolygon2D();

   private boolean visualize = true;
   private PolygonSnapperVisualizer visualizer;

   @BeforeEach
   public void setup()
   {
      visualize = visualize && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

      if(visualize)
      {
         visualizer = new PolygonSnapperVisualizer(footPolygons.get(RobotSide.LEFT));
      }

      unitSquare.addVertex(-0.5, -0.5);
      unitSquare.addVertex(0.5, -0.5);
      unitSquare.addVertex(-0.5, 0.5);
      unitSquare.addVertex(0.5, 0.5);
      unitSquare.update();
   }

   @Test
   public void testIdentity()
   {
      FootstepNode nodeToSnap = new FootstepNode(-0.3, 2.2);
      RigidBodyTransform nodeTransform = new RigidBodyTransform();
      FootstepNodeTools.getNodeTransform(nodeToSnap, nodeTransform);

      RigidBodyTransform transformToWorld = new RigidBodyTransform();

      doAFullFootholdTest(transformToWorld, nodeToSnap);
   }

   @Test
   public void testVerticalTranslation()
   {
      FootstepNode nodeToSnap = new FootstepNode(2.5, -0.5);
      RigidBodyTransform nodeTransform = new RigidBodyTransform();
      FootstepNodeTools.getNodeTransform(nodeToSnap, nodeTransform);

      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      transformToWorld.setTranslationZ(-1.0);

      doAFullFootholdTest(transformToWorld, nodeToSnap);
   }

   @Test
   public void testSimpleRotation()
   {
      FootstepNode nodeToSnap = new FootstepNode(0.0, 0.0);
      RigidBodyTransform nodeTransform = new RigidBodyTransform();
      FootstepNodeTools.getNodeTransform(nodeToSnap, nodeTransform);

      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      transformToWorld.setRotation(new AxisAngle(0.0, 1.0, 0.0, 0.25 * Math.PI));

      doAFullFootholdTest(transformToWorld, nodeToSnap);
   }

   @Test
   public void testSimpleTranslationAndRotation()
   {
      FootstepNode nodeToSnap = new FootstepNode(1.1, 0.0);
      RigidBodyTransform nodeTransform = new RigidBodyTransform();
      FootstepNodeTools.getNodeTransform(nodeToSnap, nodeTransform);

      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      transformToWorld.setRotation(new AxisAngle(0.0, 1.0, 0.0, 0.25 * Math.PI));
      transformToWorld.setTranslationZ(-1.0);

      doAFullFootholdTest(transformToWorld, nodeToSnap);
   }

   @Test
   public void testSimplePartialFoothold()
   {
      FootstepNode nodeToSnap = new FootstepNode(1.0, 0.0);
      RigidBodyTransform nodeTransform = new RigidBodyTransform();
      FootstepNodeTools.getNodeTransform(nodeToSnap, nodeTransform);

      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      transformToWorld.setRotation(new AxisAngle(0.0, 1.0, 0.0, 0.25 * Math.PI));
      transformToWorld.setTranslationZ(-1.0);

      ConvexPolygon2D partialFootholdPolygon = new ConvexPolygon2D(footPolygons.get(RobotSide.LEFT));
      partialFootholdPolygon.scale(0.5);
      partialFootholdPolygon.translate(Math.sqrt(2.0), 0.0);

      PlanarRegion planarRegion = new PlanarRegion(transformToWorld, partialFootholdPolygon);
      PlanarRegionsList planarRegionsList = new PlanarRegionsList(planarRegion);
      snapper.setPlanarRegions(planarRegionsList);
      FootstepNodeSnapData snapData = snapper.snapFootstepNode(nodeToSnap);

      if(visualize)
      {
         visualizer.addPlanarRegionsList(planarRegionsList, YoAppearance.AliceBlue());
         visualizer.setSnappedPolygon(nodeTransform, snapData.getSnapTransform(), snapData.getCroppedFoothold());
         ThreadTools.sleepForever();
      }

      assertEquals(snapData.getCroppedFoothold().getArea(), partialFootholdPolygon.getArea(), epsilon);
      PlanarRegionPolygonSnapperTest.assertSurfaceNormalsMatchAndSnapPreservesXFromAbove(snapData.getSnapTransform(), transformToWorld);
   }

   @Test
   public void testRandomFullFootholds()
   {
      int numTests = 100;
      RigidBodyTransform nodeToWorldTransform = new RigidBodyTransform();

      for (int i = 0; i < numTests; i++)
      {
         FootstepNode node = FootstepNode.generateRandomFootstepNode(random, 5.0);
         FootstepNodeTools.getNodeTransform(node, nodeToWorldTransform);

         RigidBodyTransform regionToWorld = new RigidBodyTransform(nodeToWorldTransform);
         double xRotation = EuclidCoreRandomTools.nextDouble(random, 0.15 * Math.PI);
         double yRotation = EuclidCoreRandomTools.nextDouble(random, 0.15 * Math.PI);
         regionToWorld.setRotationEuler(xRotation, yRotation, 0.0);
         regionToWorld.setTranslationZ(EuclidCoreRandomTools.nextDouble(random, 2.0));

         doAFullFootholdTest(regionToWorld, node);
      }
   }

   private void doAFullFootholdTest(RigidBodyTransform regionToWorldFrameTransform, FootstepNode nodeToSnap)
   {
      RigidBodyTransform nodeTransform = new RigidBodyTransform();
      FootstepNodeTools.getNodeTransform(nodeToSnap, nodeTransform);
      ConvexPolygon2D footholdPolygon = new ConvexPolygon2D(unitSquare);
      footholdPolygon.applyTransform(nodeTransform, false);

      PlanarRegion planarRegion = createPlanarRegion(regionToWorldFrameTransform, footholdPolygon);
      PlanarRegionsList planarRegionsList = new PlanarRegionsList(planarRegion);
      snapper.setPlanarRegions(planarRegionsList);

      FootstepNodeSnapData snapData = snapper.snapFootstepNode(nodeToSnap);

      if(visualize)
      {
         RigidBodyTransform snappedNodeTransform = new RigidBodyTransform();
         FootstepNodeTools.getSnappedNodeTransform(nodeToSnap, snapData.getSnapTransform(), snappedNodeTransform);

         visualizer.addPlanarRegionsList(planarRegionsList, YoAppearance.AliceBlue());
         visualizer.setSnappedPolygon(nodeTransform, snapData.getSnapTransform());
         ThreadTools.sleepForever();
      }

      assertEquals(snapData.getCroppedFoothold().getArea(), footPolygons.get(RobotSide.LEFT).getArea(), epsilon);
      PlanarRegionPolygonSnapperTest.assertSurfaceNormalsMatchAndSnapPreservesXFromAbove(snapData.getSnapTransform(), regionToWorldFrameTransform);
   }

   private PlanarRegion createPlanarRegion(RigidBodyTransform regionToWorldFrameTransform, ConvexPolygon2D desiredFootholdInWorldXYPlane)
   {
      RigidBodyTransform worldToRegionTransform = new RigidBodyTransform(regionToWorldFrameTransform);
      worldToRegionTransform.invert();

      ConvexPolygon2D planarRegionPolygon = new ConvexPolygon2D();
      for (int i = 0; i < desiredFootholdInWorldXYPlane.getNumberOfVertices(); i++)
      {
         Point2DReadOnly vertex = desiredFootholdInWorldXYPlane.getVertex(i);
         double zHeight = getPlaneZGivenXY(regionToWorldFrameTransform, vertex.getX(), vertex.getY());
         Vector4D transformPoint = new Vector4D(vertex.getX(), vertex.getY(), zHeight, 1.0);
         worldToRegionTransform.transform(transformPoint);
         planarRegionPolygon.addVertex(transformPoint.getX(), transformPoint.getY());
      }
      planarRegionPolygon.update();
      return new PlanarRegion(regionToWorldFrameTransform, planarRegionPolygon);
   }

   // taken from PlanarRegion, should probably be made a public static method
   public double getPlaneZGivenXY(RigidBodyTransform planeToWorldTransform, double xWorld, double yWorld)
   {
      // The three components of the plane origin
      double x0 = planeToWorldTransform.getM03();
      double y0 = planeToWorldTransform.getM13();
      double z0 = planeToWorldTransform.getM23();
      // The three components of the plane normal
      double a = planeToWorldTransform.getM02();
      double b = planeToWorldTransform.getM12();
      double c = planeToWorldTransform.getM22();

      // Given the plane equation: a*x + b*y + c*z + d = 0, with d = -(a*x0 + b*y0 + c*z0), we find z:
      double z = a / c * (x0 - xWorld) + b / c * (y0 - yWorld) + z0;
      return z;
   }

   private static void generateRandomPartialFootholdPolygon(Random random, ConvexPolygon2D footPolygon, ConvexPolygon2D partialFootholdPolygonToPack, ConvexPolygon2D planarRegionPolygonToPack)
   {
      partialFootholdPolygonToPack.clear();
      planarRegionPolygonToPack.clear();

      switch(random.nextInt(2))
      {
      case 0:
         // side edge to side edge
         partialFootholdPolygonToPack.addVertex(footPolygon.getVertex(0));
         partialFootholdPolygonToPack.addVertex(footPolygon.getVertex(1));

         double percentageAlongLine1 = EuclidCoreRandomTools.nextDouble(random, 0.1, 0.9);
         double percentageAlongLine2 = EuclidCoreRandomTools.nextDouble(random, 0.1, 0.9);
         partialFootholdPolygonToPack.addVertex(getPointAlongLineSegment(footPolygon.getVertex(1), footPolygon.getVertex(2), percentageAlongLine1));
         partialFootholdPolygonToPack.addVertex(getPointAlongLineSegment(footPolygon.getVertex(0), footPolygon.getVertex(3), percentageAlongLine2));
         break;

      case 1:
         // front edge to back edge
         partialFootholdPolygonToPack.addVertex(footPolygon.getVertex(0));
         partialFootholdPolygonToPack.addVertex(footPolygon.getVertex(3));

         percentageAlongLine1 = EuclidCoreRandomTools.nextDouble(random, 0.1, 0.9);
         percentageAlongLine2 = EuclidCoreRandomTools.nextDouble(random, 0.1, 0.9);
         partialFootholdPolygonToPack.addVertex(getPointAlongLineSegment(footPolygon.getVertex(0), footPolygon.getVertex(1), percentageAlongLine1));
         partialFootholdPolygonToPack.addVertex(getPointAlongLineSegment(footPolygon.getVertex(2), footPolygon.getVertex(3), percentageAlongLine2));
         break;
      }

      partialFootholdPolygonToPack.update();

      // TODO construct a bigger planar region
      planarRegionPolygonToPack.set(partialFootholdPolygonToPack);
      planarRegionPolygonToPack.update();
   }

   private static Point2D getPointAlongLineSegment(Point2DReadOnly point1, Point2DReadOnly point2, double percentageFromPoint1ToPoint2)
   {
      Point2D pointToReturn = new Point2D(point2);
      pointToReturn.sub(point1);
      pointToReturn.scale(percentageFromPoint1ToPoint2);
      pointToReturn.add(point1);
      return pointToReturn;
   }
}
