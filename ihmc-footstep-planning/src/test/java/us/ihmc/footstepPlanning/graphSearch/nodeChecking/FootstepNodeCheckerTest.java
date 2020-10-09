package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstanceNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.graphics.Graphics3DObjectTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

public class FootstepNodeCheckerTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final boolean visualize = simulationTestingParameters.getKeepSCSUp();

   private final SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   @Test
   public void testSwingingThroughObstacle0()
   {
      FootstepPlannerParametersReadOnly parameters = new DefaultFootstepPlannerParameters();

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(-0.5, 0.5, 0.5);
      generator.rotate(Math.PI / 2.0, Axis3D.Y);
      generator.addRectangle(1.0, 2.0);
      PlanarRegionsList planarRegions = generator.getPlanarRegionsList();

      FootstepNodeSnapAndWiggler snapper = new TestSnapper();
      FootstepNodeChecker checker = new FootstepNodeChecker(parameters, footPolygons, snapper, registry);
      checker.setPlanarRegions(planarRegions);

      FootstepNode step0 = new FootstepNode(-0.65, 0.15, 0.0, RobotSide.RIGHT);
      FootstepNode step1 = new FootstepNode(-0.65, -0.1, 0.0, RobotSide.LEFT);
      FootstepNode step2 = new FootstepNode(-0.2, 0.15, 0.0, RobotSide.RIGHT);

      snapper.addSnapData(step0, new FootstepNodeSnapData(new RigidBodyTransform()));
      snapper.addSnapData(step1, new FootstepNodeSnapData(new RigidBodyTransform()));
      snapper.addSnapData(step2, new FootstepNodeSnapData(new RigidBodyTransform()));

      if (visualize)
      {
         Graphics3DObject graphics = new Graphics3DObject();
         graphics.addCoordinateSystem(0.3);
         Graphics3DObjectTools.addPlanarRegionsList(graphics, planarRegions);

         Point3D nodeA = new Point3D(step1.getOrComputeMidFootPoint(parameters.getIdealFootstepWidth()));
         Point3D nodeB = new Point3D(step2.getOrComputeMidFootPoint(parameters.getIdealFootstepWidth()));

         PlanarRegion bodyRegion = ObstacleBetweenNodesChecker.createBodyRegionFromNodes(nodeA, nodeB, parameters.getBodyBoxBaseZ(), 2.0);
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
         graphics.translate(step1.getX(), step1.getY(), 0.0);
         graphics.addSphere(0.05, YoAppearance.Green());

         graphics.identity();
         graphics.translate(step2.getX(), step2.getY(), 0.0);
         graphics.addSphere(0.05, YoAppearance.Red());

         SimulationConstructionSet scs = new SimulationConstructionSet();
         scs.setGroundVisible(false);
         scs.addStaticLinkGraphics(graphics);

         scs.startOnAThread();
         ThreadTools.sleepForever();
      }
      else
      {
      }         Assert.assertFalse(checker.isNodeValid(step2, step1, step0));

   }

   @Test
   public void testSwingingThroughObstacle1()
   {
      FootstepPlannerParametersReadOnly parameters = new DefaultFootstepPlannerParameters();
      double bodyGroundClearance = parameters.getBodyBoxBaseZ();

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(0.0, 0.0, 0.001);
      generator.addRectangle(1.0, 1.0);
      generator.translate(0.0, 0.0, bodyGroundClearance);
      generator.rotate(Math.PI / 2.0, Axis3D.X);
      generator.addRectangle(1.0, bodyGroundClearance);
      PlanarRegionsList planarRegions = generator.getPlanarRegionsList();

      FootstepNodeSnapAndWiggler snapper = new TestSnapper();
      FootstepNodeChecker checker = new FootstepNodeChecker(parameters, footPolygons, snapper, registry);
      checker.setPlanarRegions(planarRegions);

      FootstepNode step0 = new FootstepNode(-0.1, -0.2, 0.0, RobotSide.RIGHT);
      FootstepNode step1 = new FootstepNode(-0.1, 0.25, 0.0, RobotSide.LEFT);
      FootstepNode step2 = new FootstepNode(0.1, -0.2, 0.0, RobotSide.RIGHT);

      snapper.addSnapData(step0, new FootstepNodeSnapData(new RigidBodyTransform()));
      snapper.addSnapData(step1, new FootstepNodeSnapData(new RigidBodyTransform()));
      snapper.addSnapData(step2, new FootstepNodeSnapData(new RigidBodyTransform()));

      if (visualize)
      {
         Graphics3DObject graphics = new Graphics3DObject();
         graphics.addCoordinateSystem(0.3);
         Graphics3DObjectTools.addPlanarRegionsList(graphics, planarRegions);

         Point3D nodeA = new Point3D(step1.getOrComputeMidFootPoint(parameters.getIdealFootstepWidth()));
         Point3D nodeB = new Point3D(step2.getOrComputeMidFootPoint(parameters.getIdealFootstepWidth()));
         PlanarRegion bodyRegion = ObstacleBetweenNodesChecker.createBodyRegionFromNodes(nodeA, nodeB, bodyGroundClearance, 2.0);
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
         graphics.translate(step1.getX(), step1.getY(), 0.0);
         graphics.addSphere(0.05, YoAppearance.Green());

         graphics.identity();
         graphics.translate(step2.getX(), step2.getY(), 0.0);
         graphics.addSphere(0.05, YoAppearance.Red());

         SimulationConstructionSet scs = new SimulationConstructionSet();
         scs.addStaticLinkGraphics(graphics);

         scs.startOnAThread();
         ThreadTools.sleepForever();
      }
      else
      {
         Assert.assertFalse(checker.isNodeValid(step2, step1, step0));
      }
   }

   @Test
   public void testValidNode()
   {
      FootstepNodeSnapAndWiggler snapper = new TestSnapper();
      FootstepPlannerParametersReadOnly parameters = new DefaultFootstepPlannerParameters();
      FootstepNodeChecker checker = new FootstepNodeChecker(parameters, footPolygons, snapper, registry);

      // the checker should check for limits in z-height, pitch, and roll.
      // the valid ranges for x, y, and yaw should be considered in the node expansion.
      FootstepNode step0 = new FootstepNode(0.0, -0.2, 0.0, RobotSide.RIGHT);
      FootstepNode step1 = new FootstepNode(0.0, 0.0, 0.0, RobotSide.LEFT);
      FootstepNode step2 = new FootstepNode(0.1, -0.2, 0.0, RobotSide.RIGHT);

      snapper.addSnapData(step0, FootstepNodeSnapData.identityData());
      snapper.addSnapData(step1, FootstepNodeSnapData.identityData());
      snapper.addSnapData(step2, FootstepNodeSnapData.identityData());

      Assert.assertTrue(checker.isNodeValid(step2, step1, step0));
   }

   @Test
   public void testManuallyAddedSnapDataIsValid()
   {
      FootstepNodeSnapAndWiggler snapper = new TestSnapper();
      FootstepPlannerParametersReadOnly parameters = new DefaultFootstepPlannerParameters();
      FootstepNodeChecker checker = new FootstepNodeChecker(parameters, footPolygons, snapper, registry);

      FootstepNode step0 = new FootstepNode(0.0, -0.3, 0.0, RobotSide.RIGHT);
      FootstepNode step1 = new FootstepNode(0.0, 0.0, 0.0, RobotSide.LEFT);

      snapper.addSnapData(step0, FootstepNodeSnapData.identityData());
      snapper.addSnapData(step1, FootstepNodeSnapData.identityData());

      Assert.assertTrue(checker.isNodeValid(step1, step0, null));
   }

   @Test
   public void testNodesOnSameSides()
   {
      FootstepPlannerParametersReadOnly parameters = new DefaultFootstepPlannerParameters();
      FootstepNodeChecker checker = new FootstepNodeChecker(parameters, footPolygons, new TestSnapper(), registry);
      FootstepNode leftNode0 = new FootstepNode(0.0, 0.0, 0.0, RobotSide.LEFT);
      FootstepNode leftNode1 = new FootstepNode(5.0, 0.0, 2.0, RobotSide.LEFT);
      FootstepNode rightNode0 = new FootstepNode(-1.0, 0.0, -2.5, RobotSide.RIGHT);
      FootstepNode rightNode1 = new FootstepNode(0.0, 2.0, 1.0, RobotSide.RIGHT);

      Assertions.assertThrows(RuntimeException.class, () -> checker.isNodeValid(leftNode0, leftNode0, null));
      Assertions.assertThrows(RuntimeException.class, () -> checker.isNodeValid(leftNode0, leftNode1, null));
      Assertions.assertThrows(RuntimeException.class, () -> checker.isNodeValid(rightNode0, rightNode0, null));
      Assertions.assertThrows(RuntimeException.class, () -> checker.isNodeValid(rightNode0, rightNode1, null));
      Assertions.assertThrows(RuntimeException.class, () -> checker.isNodeValid(leftNode0, rightNode0, rightNode1));
      Assertions.assertThrows(RuntimeException.class, () -> checker.isNodeValid(rightNode0, leftNode0, leftNode0));
   }

   @Test
   public void testTooHighNode()
   {
      FootstepPlannerParametersReadOnly parameters = new DefaultFootstepPlannerParameters()
      {
         @Override
         public double getMaximumLeftStepZ()
         {
            return 1.0e-10;
         };

         @Override
         public double getMaximumRightStepZ()
         {
            return 1.0e-10;
         };
      };

      FootstepNodeSnapAndWiggler snapper = new TestSnapper();
      FootstepNodeChecker checker = new FootstepNodeChecker(parameters, footPolygons, snapper, registry);

      // add planar regions, otherwise will always be valid
      PlanarRegionsListGenerator planarRegionsListGenerator = new PlanarRegionsListGenerator();
      planarRegionsListGenerator.addRectangle(1.0, 1.0);
      PlanarRegionsList dummyRegions = planarRegionsListGenerator.getPlanarRegionsList();
      checker.setPlanarRegions(dummyRegions);

      FootstepNode step0 = new FootstepNode(0.2, -0.2, 0.0, RobotSide.RIGHT);
      FootstepNode step1 = new FootstepNode(0.2, 0.2, 0.0, RobotSide.LEFT);
      RigidBodyTransform snapTransform0 = new RigidBodyTransform();

      // too high step
      FootstepNode step2 = new FootstepNode(0.0, 0.0, 0.0, RobotSide.RIGHT);
      RigidBodyTransform snapTransform1 = new RigidBodyTransform();
      snapTransform1.getTranslation().setZ(parameters.getMaximumAverageStepZ() + 1.0e-10);
      snapper.addSnapData(step1, new FootstepNodeSnapData(snapTransform0));
      snapper.addSnapData(step2, new FootstepNodeSnapData(snapTransform1));
      Assert.assertFalse(checker.isNodeValid(step2, step1, step0));

      // too low step
      // if we add different step-up vs step-down heights this will need to be adjusted
      FootstepNode step3 = new FootstepNode(0.0, 0.0, 0.0, RobotSide.RIGHT);
      RigidBodyTransform snapTransform2 = new RigidBodyTransform();
      snapTransform2.getTranslation().setZ(-parameters.getMaximumAverageStepZ() - 1.0e-10);
      snapper.addSnapData(step3, new FootstepNodeSnapData(snapTransform2));
      Assert.assertFalse(checker.isNodeValid(step3, step1, step0));

      // good step
      FootstepNode step4 = new FootstepNode(0.0, 0.0, 0.0, RobotSide.RIGHT);
      RigidBodyTransform snapTransform3 = new RigidBodyTransform();
      snapper.addSnapData(step4, new FootstepNodeSnapData(snapTransform3));
      Assert.assertTrue(checker.isNodeValid(step4, step1, step0));
   }

   @Test
   public void testTooSmallFoothold()
   {
      FootstepPlannerParametersReadOnly parameters = new DefaultFootstepPlannerParameters();
      FootstepNodeSnapAndWiggler snapper = new TestSnapper();
      FootstepNodeChecker checker = new FootstepNodeChecker(parameters, footPolygons, snapper, registry);

      // add planar regions, otherwise will always be valid
      PlanarRegionsListGenerator planarRegionsListGenerator = new PlanarRegionsListGenerator();
      planarRegionsListGenerator.addRectangle(1.0, 1.0);
      PlanarRegionsList dummyRegions = planarRegionsListGenerator.getPlanarRegionsList();
      checker.setPlanarRegions(dummyRegions);

      double minFoothold = parameters.getMinimumFootholdPercent();

      FootstepNode step0 = new FootstepNode(0.0, 0.0, 0.0, RobotSide.LEFT);
      FootstepNode step1 = new FootstepNode(0.0, -0.2, 0.0, RobotSide.RIGHT);
      FootstepNode step2 = new FootstepNode(0.2, 0.0, 0.0, RobotSide.LEFT); // node to test
      RigidBodyTransform snapTransform0 = new RigidBodyTransform();

      double footArea = footPolygons.get(step2.getRobotSide()).getArea();
      double footholdArea = footArea * minFoothold * 1.01;
      double side = Math.sqrt(footholdArea);
      ConvexPolygon2D goodFoothold = new ConvexPolygon2D();
      goodFoothold.addVertex(side / 2.0, side / 2.0);
      goodFoothold.addVertex(-side / 2.0, side / 2.0);
      goodFoothold.addVertex(side / 2.0, -side / 2.0);
      goodFoothold.addVertex(-side / 2.0, -side / 2.0);
      goodFoothold.update();

      // sufficient foothold
      snapper.addSnapData(step2, new FootstepNodeSnapData(snapTransform0, goodFoothold));
      Assert.assertTrue(checker.isNodeValid(step2, step1, step0));

      footholdArea = footArea * minFoothold * 0.99;
      side = Math.sqrt(footholdArea);
      ConvexPolygon2D badFoothold = new ConvexPolygon2D();
      badFoothold.addVertex(side / 2.0, side / 2.0);
      badFoothold.addVertex(-side / 2.0, side / 2.0);
      badFoothold.addVertex(side / 2.0, -side / 2.0);
      badFoothold.addVertex(-side / 2.0, -side / 2.0);
      badFoothold.update();

      // too small foothold
      snapper.addSnapData(step2, new FootstepNodeSnapData(snapTransform0, badFoothold));
      Assert.assertFalse(checker.isNodeValid(step2, step1, step0));
   }

   private static final double barelyTooSteepEpsilon = 1.0e-5;
   private static final int iters = 10000;

   @Test
   public void testSnappingToIncline()
   {
      FootstepPlannerParametersBasics parameters = new DefaultFootstepPlannerParameters()
      {
         // don't use 45
         @Override
         public double getMinimumSurfaceInclineRadians()
         {
            return Math.toRadians(37.0);
         }

         @Override
         public boolean getWiggleWhilePlanning()
         {
            return true;
         }
      };

      FootstepNodeSnapAndWiggler snapper = new FootstepNodeSnapAndWiggler(footPolygons, parameters);
      testSnappingToInclinedPlane(parameters, snapper);
   }

   @Test
   public void testSnapAndWiggleOnIncline()
   {
      FootstepPlannerParametersBasics parameters = new DefaultFootstepPlannerParameters()
      {
         // don't use 45
         @Override
         public double getMinimumSurfaceInclineRadians()
         {
            return Math.toRadians(37.0);
         }

         @Override
         public boolean getWiggleWhilePlanning()
         {
            return true;
         }
      };

      FootstepNodeSnapAndWiggler snapper = new FootstepNodeSnapAndWiggler(footPolygons, parameters);
      testSnappingToInclinedPlane(parameters, snapper);
   }

   public void testSnappingToInclinedPlane(FootstepPlannerParametersBasics parameters, FootstepNodeSnapAndWiggler snapper)
   {
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(1.0, 1.0);
      polygon.addVertex(1.0, -1.0);
      polygon.addVertex(-1.0, -1.0);
      polygon.addVertex(-1.0, 1.0);
      polygon.update();
      ArrayList<ConvexPolygon2D> polygons = new ArrayList<>();
      polygons.add(polygon);

      PlanarRegion planarRegion = new PlanarRegion();
      planarRegion.set(transformToWorld, polygons);

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(planarRegion);

      double footLength = 0.2;
      double footWidth = 0.1;
      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createFootPolygons(footLength, footWidth);

      FootstepNodeChecker nodeChecker = new FootstepNodeChecker(parameters, footPolygons, snapper, registry);
      nodeChecker.setPlanarRegions(planarRegionsList);

      FootstepNode step0 = new FootstepNode(-0.1, -0.1, 0.0, RobotSide.RIGHT);
      FootstepNode step1 = new FootstepNode(0.0, 0.1, 0.0, RobotSide.LEFT);
      FootstepNode step2 = new FootstepNode(0.0, -0.1, 0.0, RobotSide.RIGHT);

      // TODO add getter for rejection reason or retreive from
      assertTrue(nodeChecker.isNodeValid(step2, step1, step0));
//      assertEquals(null, nodeChecker.getRejectionReason());

      double rotationAngle;

      // test a bunch of independent roll/pitch valid angles
      for (rotationAngle = -parameters.getMinimumSurfaceInclineRadians() + barelyTooSteepEpsilon; rotationAngle < parameters.getMinimumSurfaceInclineRadians() - barelyTooSteepEpsilon; rotationAngle += 0.001)
      {
         transformToWorld.setIdentity();
         transformToWorld.appendRollRotation(rotationAngle);
         planarRegion.set(transformToWorld, polygons);
         nodeChecker.setPlanarRegions(planarRegionsList);
         snapper.setPlanarRegions(planarRegionsList);

         Assert.assertTrue(nodeChecker.isNodeValid(step2, step1, step0));
//         assertEquals(null, registry.getRejectionReason());

         transformToWorld.setIdentity();
         transformToWorld.appendPitchRotation(rotationAngle);
         planarRegion.set(transformToWorld, polygons);
         nodeChecker.setPlanarRegions(planarRegionsList);

         Assert.assertTrue(nodeChecker.isNodeValid(step2, step1, step0));
//         assertEquals(null, registry.getRejectionReason());
      }

      for (rotationAngle = parameters.getMinimumSurfaceInclineRadians() + 0.001; rotationAngle < Math.toRadians(75); rotationAngle += 0.001)
      {
         transformToWorld.setIdentity();
         transformToWorld.appendRollRotation(rotationAngle);
         planarRegion.set(transformToWorld, polygons);
         snapper.setPlanarRegions(planarRegionsList);
         nodeChecker.setPlanarRegions(planarRegionsList);

         assertFalse("rotation = " + rotationAngle, nodeChecker.isNodeValid(step2, step1, step0));
//         assertEquals("rotation = " + rotationAngle, BipedalFootstepPlannerNodeRejectionReason.SURFACE_NORMAL_TOO_STEEP_TO_SNAP, registry.getRejectionReason());

         transformToWorld.setIdentity();
         transformToWorld.appendPitchRotation(rotationAngle);
         planarRegion.set(transformToWorld, polygons);
         nodeChecker.setPlanarRegions(planarRegionsList);

         assertFalse("rotation = " + rotationAngle, nodeChecker.isNodeValid(step2, step1, step0));
//         assertEquals("rotation = " + rotationAngle, BipedalFootstepPlannerNodeRejectionReason.SURFACE_NORMAL_TOO_STEEP_TO_SNAP, registry.getRejectionReason());
      }

      for (rotationAngle = -Math.toRadians(75); rotationAngle < -parameters.getMinimumSurfaceInclineRadians(); rotationAngle += 0.001)
      {
         transformToWorld.setIdentity();
         transformToWorld.appendRollRotation(rotationAngle);
         planarRegion.set(transformToWorld, polygons);
         nodeChecker.setPlanarRegions(planarRegionsList);
         snapper.setPlanarRegions(planarRegionsList);

         assertFalse("rotation = " + rotationAngle, nodeChecker.isNodeValid(step2, step1, step0));
//         assertEquals("rotation = " + rotationAngle, BipedalFootstepPlannerNodeRejectionReason.SURFACE_NORMAL_TOO_STEEP_TO_SNAP, registry.getRejectionReason());

         transformToWorld.setIdentity();
         transformToWorld.appendPitchRotation(rotationAngle);
         planarRegion.set(transformToWorld, polygons);
         nodeChecker.setPlanarRegions(planarRegionsList);

         assertFalse("rotation = " + rotationAngle, nodeChecker.isNodeValid(step2, step1, step0));
//         assertEquals("rotation = " + rotationAngle, BipedalFootstepPlannerNodeRejectionReason.SURFACE_NORMAL_TOO_STEEP_TO_SNAP, registry.getRejectionReason());
      }

      // test random orientations
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         QuaternionReadOnly orientation3DReadOnly = EuclidCoreRandomTools.nextQuaternion(random);

         transformToWorld.setIdentity();
         transformToWorld.getRotation().set(orientation3DReadOnly);
         planarRegion.set(transformToWorld, polygons);
         nodeChecker.setPlanarRegions(planarRegionsList);
         snapper.setPlanarRegions(planarRegionsList);

         Vector3D vertical = new Vector3D(0.0, 0.0, 1.0);
         Vector3D normal = new Vector3D(vertical);
         orientation3DReadOnly.transform(normal);
         //         QuaternionReadOnly noOrientation = new Quaternion(orientation3DReadOnly.getYaw(), 0.0, 0.0);

         double angleFromFlat = vertical.angle(normal);
         if (Math.abs(angleFromFlat) > parameters.getMinimumSurfaceInclineRadians())
         {
            String message = "actual rotation = " + angleFromFlat + ", allowed rotation = " + parameters.getMinimumSurfaceInclineRadians();
            assertFalse(message, nodeChecker.isNodeValid(step2, step1, step0));
//            boolean correctRejection = BipedalFootstepPlannerNodeRejectionReason.SURFACE_NORMAL_TOO_STEEP_TO_SNAP == registry.getRejectionReason() ||
//                                       BipedalFootstepPlannerNodeRejectionReason.COULD_NOT_SNAP == registry.getRejectionReason();
//            assertTrue(message, correctRejection);
         }
         else
         {
            assertTrue(nodeChecker.isNodeValid(step2, step1, step0));
            //            assertEquals(null, registry.getRejectionReason());
         }
      }
   }

   @Test
   public void testWiggleMinimumThreshold()
   {
      DefaultFootstepPlannerParameters footstepPlannerParameters = new DefaultFootstepPlannerParameters();
      footstepPlannerParameters.setWiggleWhilePlanning(true);

      double wiggleMinimum = 0.02;
      footstepPlannerParameters.setWiggleInsideDeltaMinimum(wiggleMinimum);

      double wiggleTarget = 0.07;
      footstepPlannerParameters.setWiggleInsideDeltaTarget(wiggleTarget);

      double maxXYWiggle = 0.1;
      footstepPlannerParameters.setMaximumXYWiggleDistance(maxXYWiggle);

      double maxYawWiggle = 0.7;
      footstepPlannerParameters.setMaximumYawWiggle(maxYawWiggle);

      double footLength = 0.2;
      double footWidth = 0.1;
      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createFootPolygons(footLength, footWidth);

      FootstepNodeSnapAndWiggler snapper = new FootstepNodeSnapAndWiggler(footPolygons, footstepPlannerParameters);
      FootstepNodeChecker checker = new FootstepNodeChecker(footstepPlannerParameters, footPolygons, snapper, new YoRegistry("testRegistry"));

      FootstepNode step0 = new FootstepNode(0.0, -0.2, 0.0, RobotSide.RIGHT);
      FootstepNode step1 = new FootstepNode(0.0, 0.1, 0.0, RobotSide.LEFT);

      PlanarRegionsListGenerator planarRegionsListGenerator = new PlanarRegionsListGenerator();
      double regionEpsilon = 1e-5;

      // test just enough area
      double regionX = footLength + 2.0 * wiggleMinimum + regionEpsilon;
      double regionY = footWidth + 2.0 * wiggleMinimum + regionEpsilon;
      planarRegionsListGenerator.reset();

      planarRegionsListGenerator.translate(0.5 * maxXYWiggle, 0.5 * maxXYWiggle, 0.1);
      planarRegionsListGenerator.addRectangle(regionX, regionY);
      PlanarRegionsList regions = planarRegionsListGenerator.getPlanarRegionsList();
      snapper.setPlanarRegions(regions);
      checker.setPlanarRegions(regions);
      snapper.initialize();
      snapper.addSnapData(step0, new FootstepNodeSnapData(new RigidBodyTransform()));
      boolean valid = checker.isNodeValid(step1, step0, null);
      System.out.println(valid);

      // test just too little area along x
      snapper.reset();
      regionX = footLength + 2.0 * wiggleMinimum - regionEpsilon;
      regionY = footWidth + 2.0 * wiggleMinimum + regionEpsilon;
      planarRegionsListGenerator.reset();

      planarRegionsListGenerator.translate(0.5 * maxXYWiggle, 0.5 * maxXYWiggle, 0.1);
      planarRegionsListGenerator.addRectangle(regionX, regionY);
      regions = planarRegionsListGenerator.getPlanarRegionsList();
      snapper.setPlanarRegions(regions);
      checker.setPlanarRegions(regions);
      snapper.initialize();
      snapper.addSnapData(step0, new FootstepNodeSnapData(new RigidBodyTransform()));
      valid = checker.isNodeValid(step1, step0, null);
      System.out.println(valid);

      // test just too little area along y
      snapper.reset();
      regionX = footLength + 2.0 * wiggleMinimum + regionEpsilon;
      regionY = footWidth + 2.0 * wiggleMinimum - regionEpsilon;
      planarRegionsListGenerator.reset();

      planarRegionsListGenerator.translate(0.5 * maxXYWiggle, 0.5 * maxXYWiggle, 0.1);
      planarRegionsListGenerator.addRectangle(regionX, regionY);
      regions = planarRegionsListGenerator.getPlanarRegionsList();
      snapper.setPlanarRegions(regions);
      checker.setPlanarRegions(regions);
      snapper.initialize();
      snapper.addSnapData(step0, new FootstepNodeSnapData(new RigidBodyTransform()));
      valid = checker.isNodeValid(step1, step0, null);
      System.out.println(valid);
   }

   private class TestSnapper extends FootstepNodeSnapAndWiggler
   {
      public TestSnapper()
      {
         super(PlannerTools.createDefaultFootPolygons(), new DefaultFootstepPlannerParameters());
      }

      @Override
      protected FootstepNodeSnapData computeSnapTransform(FootstepNode footstepNode, FootstepNode stanceNode)
      {
         throw new RuntimeException("In this test snapper add nodes manually.");
      }
   }
}
