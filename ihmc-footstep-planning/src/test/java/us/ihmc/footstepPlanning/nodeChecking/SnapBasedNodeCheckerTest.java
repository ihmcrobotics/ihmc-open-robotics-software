package us.ihmc.footstepPlanning.nodeChecking;

import java.util.List;

import org.junit.Assert;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TestName;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.footstepPlanning.DefaultFootstepPlanningParameters;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.SnapBasedNodeChecker;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.graphics.Graphics3DObjectTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;

public class SnapBasedNodeCheckerTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final boolean visualize = simulationTestingParameters.getKeepSCSUp();

   private final SideDependentList<ConvexPolygon2D> footPolygons = PlanningTestTools.createDefaultFootPolygons();

   @Rule
   public TestName name = new TestName();

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSwingingThroughObstacle0()
   {
      FootstepPlannerParameters parameters = new DefaultFootstepPlanningParameters();

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(-0.5, 0.5, 0.5);
      generator.rotate(Math.PI / 2.0, Axis.Y);
      generator.addRectangle(1.0, 2.0);
      PlanarRegionsList planarRegions = generator.getPlanarRegionsList();

      FootstepNodeSnapper snapper = new TestSnapper();
      SnapBasedNodeChecker checker = new SnapBasedNodeChecker(parameters, footPolygons, snapper);
      checker.setPlanarRegions(planarRegions);

      FootstepNode node0 = new FootstepNode(-0.65, -0.1, 0.0, RobotSide.LEFT);
      FootstepNode node1 = new FootstepNode(-0.2, 0.15, 0.0, RobotSide.RIGHT);
      snapper.addSnapData(node0, new FootstepNodeSnapData(new RigidBodyTransform()));
      snapper.addSnapData(node1, new FootstepNodeSnapData(new RigidBodyTransform()));

      if (visualize)
      {
         Graphics3DObject graphics = new Graphics3DObject();
         graphics.addCoordinateSystem(0.3);
         Graphics3DObjectTools.addPlanarRegionsList(graphics, planarRegions);

         Point3D nodeA = new Point3D(node0.getOrComputeMidFootPoint(parameters.getIdealFootstepWidth()));
         Point3D nodeB = new Point3D(node1.getOrComputeMidFootPoint(parameters.getIdealFootstepWidth()));

         PlanarRegion bodyRegion = SnapBasedNodeChecker.createBodyRegionFromNodes(nodeA, nodeB, parameters.getBodyGroundClearance(), 2.0);
         Graphics3DObjectTools.addPlanarRegionsList(graphics, new PlanarRegionsList(bodyRegion), YoAppearance.White());

         for (PlanarRegion region : planarRegions.getPlanarRegionsAsList())
         {
            List<LineSegment3D> intersections = region.intersect(bodyRegion);

            for (LineSegment3D intersection : intersections)
            {
               graphics.identity();
               graphics.translate(intersection.getFirstEndpoint());
               Vector3D zAxis = new Vector3D(0.0, 0.0, 1.0);
               Vector3DBasics direction = intersection.getDirection(true);
               double dotProduct = zAxis.dot(direction);
               Vector3D rotationAxis = new Vector3D();
               rotationAxis.cross(zAxis, direction);
               if (rotationAxis.length() < 1.0e-5)
               {
                  rotationAxis.setX(1.0);
               }
               double rotationAngle = Math.acos(dotProduct);
               graphics.rotate(rotationAngle, rotationAxis);
               graphics.addCylinder(intersection.length(), 0.02, YoAppearance.Red());
            }
         }

         graphics.identity();
         graphics.translate(node0.getX(), node0.getY(), 0.0);
         graphics.addSphere(0.05, YoAppearance.Green());

         graphics.identity();
         graphics.translate(node1.getX(), node1.getY(), 0.0);
         graphics.addSphere(0.05, YoAppearance.Red());

         SimulationConstructionSet scs = new SimulationConstructionSet();
         scs.setGroundVisible(false);
         scs.addStaticLinkGraphics(graphics);

         scs.startOnAThread();
         ThreadTools.sleepForever();
      }
      else
      {
         Assert.assertFalse(checker.isNodeValid(node0, node1));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSwingingThroughObstacle1()
   {
      FootstepPlannerParameters parameters = new DefaultFootstepPlanningParameters();
      double bodyGroundClearance = parameters.getBodyGroundClearance();

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(0.0, 0.0, 0.001);
      generator.addRectangle(1.0, 1.0);
      generator.translate(0.0, 0.0, bodyGroundClearance);
      generator.rotate(Math.PI / 2.0, Axis.X);
      generator.addRectangle(1.0, bodyGroundClearance);
      PlanarRegionsList planarRegions = generator.getPlanarRegionsList();

      FootstepNodeSnapper snapper = new TestSnapper();
      SnapBasedNodeChecker checker = new SnapBasedNodeChecker(parameters, footPolygons, snapper);
      checker.setPlanarRegions(planarRegions);

      FootstepNode node0 = new FootstepNode(-0.1, 0.25, 0.0, RobotSide.LEFT);
      FootstepNode node1 = new FootstepNode(0.1, -0.2, 0.0, RobotSide.RIGHT);
      snapper.addSnapData(node0, new FootstepNodeSnapData(new RigidBodyTransform()));
      snapper.addSnapData(node1, new FootstepNodeSnapData(new RigidBodyTransform()));

      if (visualize)
      {
         Graphics3DObject graphics = new Graphics3DObject();
         graphics.addCoordinateSystem(0.3);
         Graphics3DObjectTools.addPlanarRegionsList(graphics, planarRegions);

         Point3D nodeA = new Point3D(node0.getOrComputeMidFootPoint(parameters.getIdealFootstepWidth()));
         Point3D nodeB = new Point3D(node1.getOrComputeMidFootPoint(parameters.getIdealFootstepWidth()));
         PlanarRegion bodyRegion = SnapBasedNodeChecker.createBodyRegionFromNodes(nodeA, nodeB, parameters.getBodyGroundClearance(), 2.0);
         Graphics3DObjectTools.addPlanarRegionsList(graphics, new PlanarRegionsList(bodyRegion), YoAppearance.White());

         for (PlanarRegion region : planarRegions.getPlanarRegionsAsList())
         {
            List<LineSegment3D> intersections = region.intersect(bodyRegion);
            for (LineSegment3D intersection : intersections)
            {
               graphics.identity();
               graphics.translate(intersection.getFirstEndpoint());
               Vector3D zAxis = new Vector3D(0.0, 0.0, 1.0);
               Vector3DBasics direction = intersection.getDirection(true);
               double dotProduct = zAxis.dot(direction);
               Vector3D rotationAxis = new Vector3D();
               rotationAxis.cross(zAxis, direction);
               if (rotationAxis.length() < 1.0e-5)
               {
                  rotationAxis.setX(1.0);
               }
               double rotationAngle = Math.acos(dotProduct);
               graphics.rotate(rotationAngle, rotationAxis);
               graphics.addCylinder(intersection.length(), 0.02, YoAppearance.Red());
            }
         }

         graphics.identity();
         graphics.translate(node0.getX(), node0.getY(), 0.0);
         graphics.addSphere(0.05, YoAppearance.Green());

         graphics.identity();
         graphics.translate(node1.getX(), node1.getY(), 0.0);
         graphics.addSphere(0.05, YoAppearance.Red());

         SimulationConstructionSet scs = new SimulationConstructionSet();
         scs.addStaticLinkGraphics(graphics);

         scs.startOnAThread();
         ThreadTools.sleepForever();
      }
      else
      {
         Assert.assertFalse(checker.isNodeValid(node0, node1));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testValidNode()
   {
      FootstepNodeSnapper snapper = new TestSnapper();
      FootstepPlannerParameters parameters = new DefaultFootstepPlanningParameters();
      SnapBasedNodeChecker checker = new SnapBasedNodeChecker(parameters, footPolygons, snapper);

      // the checker should check for limits in z-height, pitch, and roll.
      // the valid ranges for x, y, and yaw should be considered in the node expansion.
      FootstepNode node0 = new FootstepNode(0.0, 0.0, 0.0, RobotSide.LEFT);
      FootstepNode node1 = new FootstepNode(0.0, 0.0, 0.0, RobotSide.RIGHT);

      Assert.assertTrue(checker.isNodeValid(node0, node1));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testStartNodeValid()
   {
      FootstepNodeSnapper snapper = new TestSnapper();
      FootstepPlannerParameters parameters = new DefaultFootstepPlanningParameters();
      SnapBasedNodeChecker checker = new SnapBasedNodeChecker(parameters, footPolygons, snapper);

      // the start node is valid even if it can not be snapped or the parent is null.
      checker.setPlanarRegions(new PlanarRegionsList());
      FootstepNode node = new FootstepNode(0.0, 0.0, 0.0, RobotSide.LEFT);
      checker.addStartNode(node, new RigidBodyTransform());
      Assert.assertTrue(checker.isNodeValid(node, null));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public void testSameNodes()
   {
      FootstepPlannerParameters parameters = new DefaultFootstepPlanningParameters();
      SnapBasedNodeChecker checker = new SnapBasedNodeChecker(parameters, footPolygons, new TestSnapper());
      FootstepNode node = new FootstepNode(0.0, 0.0, 0.0, RobotSide.LEFT);

      // the checker should not allow checking a node against itself since this
      // is likely caused by a mistake inside the planning logic.
      checker.isNodeValid(node, node);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTooHighNode()
   {
      FootstepPlannerParameters parameters = new DefaultFootstepPlanningParameters()
      {
         @Override
         public double getMaximumStepZ()
         {
            return 1.0e-10;
         };
      };

      FootstepNodeSnapper snapper = new TestSnapper();
      SnapBasedNodeChecker checker = new SnapBasedNodeChecker(parameters, footPolygons, snapper);

      FootstepNode node0 = new FootstepNode(1.0, 0.0, 0.0, RobotSide.LEFT);
      RigidBodyTransform snapTransform0 = new RigidBodyTransform();

      // too high step
      FootstepNode node1 = new FootstepNode(0.0, 0.0, 0.0, RobotSide.RIGHT);
      RigidBodyTransform snapTransform1 = new RigidBodyTransform();
      snapTransform1.setTranslationZ(parameters.getMaximumStepZ() + 1.0e-10);
      snapper.addSnapData(node0, new FootstepNodeSnapData(snapTransform0));
      snapper.addSnapData(node1, new FootstepNodeSnapData(snapTransform1));
      Assert.assertFalse(checker.isNodeValid(node0, node1));

      // too low step
      // if we add different step-up vs step-down heights this will need to be adjusted
      FootstepNode node2 = new FootstepNode(0.0, 0.0, 0.0, RobotSide.RIGHT);
      RigidBodyTransform snapTransform2 = new RigidBodyTransform();
      snapTransform2.setTranslationZ(-parameters.getMaximumStepZ() - 1.0e-10);
      snapper.addSnapData(node2, new FootstepNodeSnapData(snapTransform2));
      Assert.assertFalse(checker.isNodeValid(node0, node2));

      // good step
      FootstepNode node3 = new FootstepNode(0.0, 0.0, 0.0, RobotSide.RIGHT);
      RigidBodyTransform snapTransform3 = new RigidBodyTransform();
      snapper.addSnapData(node3, new FootstepNodeSnapData(snapTransform3));
      Assert.assertTrue(checker.isNodeValid(node0, node3));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTooSmallFoothold()
   {
      FootstepPlannerParameters parameters = new DefaultFootstepPlanningParameters();
      FootstepNodeSnapper snapper = new TestSnapper();
      SnapBasedNodeChecker checker = new SnapBasedNodeChecker(parameters, footPolygons, snapper);

      double minFoothold = parameters.getMinimumFootholdPercent();

      FootstepNode node0 = new FootstepNode(1.0, 0.0, 0.0, RobotSide.LEFT);
      FootstepNode node1 = new FootstepNode(0.0, 0.0, 0.0, RobotSide.RIGHT);
      RigidBodyTransform snapTransform0 = new RigidBodyTransform();

      double footArea = footPolygons.get(node0.getRobotSide()).getArea();
      double footholdArea = footArea * minFoothold * 1.01;
      double side = Math.sqrt(footholdArea);
      ConvexPolygon2D goodFoothold = new ConvexPolygon2D();
      goodFoothold.addVertex(side / 2.0, side / 2.0);
      goodFoothold.addVertex(-side / 2.0, side / 2.0);
      goodFoothold.addVertex(side / 2.0, -side / 2.0);
      goodFoothold.addVertex(-side / 2.0, -side / 2.0);
      goodFoothold.update();

      // sufficient foothold
      snapper.addSnapData(node0, new FootstepNodeSnapData(snapTransform0, goodFoothold));
      Assert.assertTrue(checker.isNodeValid(node0, node1));

      footholdArea = footArea * minFoothold * 0.99;
      side = Math.sqrt(footholdArea);
      ConvexPolygon2D badFoothold = new ConvexPolygon2D();
      badFoothold.addVertex(side / 2.0, side / 2.0);
      badFoothold.addVertex(-side / 2.0, side / 2.0);
      badFoothold.addVertex(side / 2.0, -side / 2.0);
      badFoothold.addVertex(-side / 2.0, -side / 2.0);
      badFoothold.update();

      // too small foothold
      snapper.addSnapData(node0, new FootstepNodeSnapData(snapTransform0, badFoothold));
      Assert.assertFalse(checker.isNodeValid(node0, node1));
   }

   private class TestSnapper extends FootstepNodeSnapper
   {

      @Override
      protected FootstepNodeSnapData snapInternal(FootstepNode footstepNode)
      {
         throw new RuntimeException("In this test snapper add nodes manually.");
      }

   }
}
