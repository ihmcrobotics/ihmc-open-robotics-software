package us.ihmc.footstepPlanning.graphSearch.stepChecking;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.tools.EuclidCoreMissingRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.PlanarRegionFootstepSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.graphics.Graphics3DObjectTools;
import us.ihmc.commons.robotics.robotSide.RobotSide;
import us.ihmc.commons.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class FootstepCheckerTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final boolean visualize = simulationTestingParameters.getKeepSCSUp();

   private final SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   @Test
   public void testSwingingThroughObstacle0()
   {
      DefaultFootstepPlannerParametersReadOnly parameters = new DefaultFootstepPlannerParameters();

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(-0.5, 0.5, 0.5);
      generator.rotate(Math.PI / 2.0, Axis3D.Y);
      generator.addRectangle(1.0, 2.0);
      PlanarRegionsList planarRegions = generator.getPlanarRegionsList();

      PlanarRegionFootstepSnapAndWiggler snapper = new TestSnapper();
      FootstepChecker checker = new FootstepChecker(parameters, footPolygons, snapper, null, registry);
      checker.setPlanarRegions(planarRegions);

      DiscreteFootstep step0 = new DiscreteFootstep(-0.65, 0.15, 0.0, RobotSide.RIGHT);
      DiscreteFootstep step1 = new DiscreteFootstep(-0.65, -0.1, 0.0, RobotSide.LEFT);
      DiscreteFootstep step2 = new DiscreteFootstep(-0.2, 0.15, 0.0, RobotSide.RIGHT);

      snapper.addSnapData(step0, new FootstepSnapData(new RigidBodyTransform()));
      snapper.addSnapData(step1, new FootstepSnapData(new RigidBodyTransform()));
      snapper.addSnapData(step2, new FootstepSnapData(new RigidBodyTransform()));

      if (visualize)
      {
         Graphics3DObject graphics = new Graphics3DObject();
         graphics.addCoordinateSystem(0.3);
         Graphics3DObjectTools.addPlanarRegionsList(graphics, planarRegions);

         Point3D nodeA = new Point3D(step1.getOrComputeMidFootPoint(parameters.getIdealFootstepWidth()));
         Point3D nodeB = new Point3D(step2.getOrComputeMidFootPoint(parameters.getIdealFootstepWidth()));

         PlanarRegion bodyRegion = PlanarRegionObstacleBetweenStepsChecker.createBodyRegionFromSteps(nodeA, nodeB, parameters.getBodyBoxBaseZ(), 2.0);
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
      }         assertFalse(checker.isStepValid(step2, step1, step0));

   }

   @Test
   public void testSwingingThroughObstacle1()
   {
      DefaultFootstepPlannerParametersReadOnly parameters = new DefaultFootstepPlannerParameters();
      double bodyGroundClearance = parameters.getBodyBoxBaseZ();

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(0.0, 0.0, 0.001);
      generator.addRectangle(1.0, 1.0);
      generator.translate(0.0, 0.0, bodyGroundClearance);
      generator.rotate(Math.PI / 2.0, Axis3D.X);
      generator.addRectangle(1.0, bodyGroundClearance);
      PlanarRegionsList planarRegions = generator.getPlanarRegionsList();

      PlanarRegionFootstepSnapAndWiggler snapper = new TestSnapper();
      FootstepChecker checker = new FootstepChecker(parameters, footPolygons, snapper, null, registry);
      checker.setPlanarRegions(planarRegions);

      DiscreteFootstep step0 = new DiscreteFootstep(-0.1, -0.2, 0.0, RobotSide.RIGHT);
      DiscreteFootstep step1 = new DiscreteFootstep(-0.1, 0.25, 0.0, RobotSide.LEFT);
      DiscreteFootstep step2 = new DiscreteFootstep(0.1, -0.2, 0.0, RobotSide.RIGHT);

      snapper.addSnapData(step0, new FootstepSnapData(new RigidBodyTransform()));
      snapper.addSnapData(step1, new FootstepSnapData(new RigidBodyTransform()));
      snapper.addSnapData(step2, new FootstepSnapData(new RigidBodyTransform()));

      if (visualize)
      {
         Graphics3DObject graphics = new Graphics3DObject();
         graphics.addCoordinateSystem(0.3);
         Graphics3DObjectTools.addPlanarRegionsList(graphics, planarRegions);

         Point3D nodeA = new Point3D(step1.getOrComputeMidFootPoint(parameters.getIdealFootstepWidth()));
         Point3D nodeB = new Point3D(step2.getOrComputeMidFootPoint(parameters.getIdealFootstepWidth()));
         PlanarRegion bodyRegion = PlanarRegionObstacleBetweenStepsChecker.createBodyRegionFromSteps(nodeA, nodeB, bodyGroundClearance, 2.0);
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
         assertFalse(checker.isStepValid(step2, step1, step0));
      }
   }

   @Test
   public void testValidNode()
   {
      PlanarRegionFootstepSnapAndWiggler snapper = new TestSnapper();
      DefaultFootstepPlannerParametersReadOnly parameters = new DefaultFootstepPlannerParameters();
      FootstepChecker checker = new FootstepChecker(parameters, footPolygons, snapper, null, registry);

      // the checker should check for limits in z-height, pitch, and roll.
      // the valid ranges for x, y, and yaw should be considered in the node expansion.
      DiscreteFootstep step0 = new DiscreteFootstep(0.0, -0.2, 0.0, RobotSide.RIGHT);
      DiscreteFootstep step1 = new DiscreteFootstep(0.0, 0.0, 0.0, RobotSide.LEFT);
      DiscreteFootstep step2 = new DiscreteFootstep(0.1, -0.2, 0.0, RobotSide.RIGHT);

      snapper.addSnapData(step0, FootstepSnapData.identityData());
      snapper.addSnapData(step1, FootstepSnapData.identityData());
      snapper.addSnapData(step2, FootstepSnapData.identityData());

      Assert.assertTrue(checker.isStepValid(step2, step1, step0));
   }

   @Test
   public void testManuallyAddedSnapDataIsValid()
   {
      PlanarRegionFootstepSnapAndWiggler snapper = new TestSnapper();
      DefaultFootstepPlannerParametersReadOnly parameters = new DefaultFootstepPlannerParameters();
      FootstepChecker checker = new FootstepChecker(parameters, footPolygons, snapper, null, registry);

      DiscreteFootstep step0 = new DiscreteFootstep(0.0, -0.3, 0.0, RobotSide.RIGHT);
      DiscreteFootstep step1 = new DiscreteFootstep(0.0, 0.0, 0.0, RobotSide.LEFT);

      snapper.addSnapData(step0, FootstepSnapData.identityData());
      snapper.addSnapData(step1, FootstepSnapData.identityData());

      assertTrue(checker.isStepValid(step1, step0, null));
   }

   @Test
   public void testNodesOnSameSides()
   {
      DefaultFootstepPlannerParametersReadOnly parameters = new DefaultFootstepPlannerParameters();
      FootstepChecker checker = new FootstepChecker(parameters, footPolygons, new TestSnapper(), null, registry);
      DiscreteFootstep leftNode0 = new DiscreteFootstep(0.0, 0.0, 0.0, RobotSide.LEFT);
      DiscreteFootstep leftNode1 = new DiscreteFootstep(5.0, 0.0, 2.0, RobotSide.LEFT);
      DiscreteFootstep rightNode0 = new DiscreteFootstep(-1.0, 0.0, -2.5, RobotSide.RIGHT);
      DiscreteFootstep rightNode1 = new DiscreteFootstep(0.0, 2.0, 1.0, RobotSide.RIGHT);

      Assertions.assertThrows(RuntimeException.class, () -> checker.isStepValid(leftNode0, leftNode0, null));
      Assertions.assertThrows(RuntimeException.class, () -> checker.isStepValid(leftNode0, leftNode1, null));
      Assertions.assertThrows(RuntimeException.class, () -> checker.isStepValid(rightNode0, rightNode0, null));
      Assertions.assertThrows(RuntimeException.class, () -> checker.isStepValid(rightNode0, rightNode1, null));
      Assertions.assertThrows(RuntimeException.class, () -> checker.isStepValid(leftNode0, rightNode0, rightNode1));
      Assertions.assertThrows(RuntimeException.class, () -> checker.isStepValid(rightNode0, leftNode0, leftNode0));
   }

   @Test
   public void testTooHighNode()
   {
      DefaultFootstepPlannerParametersReadOnly parameters = new DefaultFootstepPlannerParameters()
      {
         @Override
         public double getMaxStepZ()
         {
            return 1.0e-10;
         };
      };

      PlanarRegionFootstepSnapAndWiggler snapper = new TestSnapper();
      FootstepChecker checker = new FootstepChecker(parameters, footPolygons, snapper, null, registry);

      // add planar regions, otherwise will always be valid
      PlanarRegionsListGenerator planarRegionsListGenerator = new PlanarRegionsListGenerator();
      planarRegionsListGenerator.addRectangle(1.0, 1.0);
      PlanarRegionsList dummyRegions = planarRegionsListGenerator.getPlanarRegionsList();
      checker.setPlanarRegions(dummyRegions);

      DiscreteFootstep step0 = new DiscreteFootstep(0.2, -0.2, 0.0, RobotSide.RIGHT);
      DiscreteFootstep step1 = new DiscreteFootstep(0.2, 0.2, 0.0, RobotSide.LEFT);
      RigidBodyTransform snapTransform0 = new RigidBodyTransform();

      // too high step
      DiscreteFootstep step2 = new DiscreteFootstep(0.0, 0.0, 0.0, RobotSide.RIGHT);
      RigidBodyTransform snapTransform1 = new RigidBodyTransform();
      snapTransform1.getTranslation().setZ(parameters.getMaxStepZ() + 1.0e-10);
      snapper.addSnapData(step1, new FootstepSnapData(snapTransform0));
      snapper.addSnapData(step2, new FootstepSnapData(snapTransform1));
      assertFalse(checker.isStepValid(step2, step1, step0));

      // too low step
      // if we add different step-up vs step-down heights this will need to be adjusted
      DiscreteFootstep step3 = new DiscreteFootstep(0.0, 0.0, 0.0, RobotSide.RIGHT);
      RigidBodyTransform snapTransform2 = new RigidBodyTransform();
      snapTransform2.getTranslation().setZ(-parameters.getMaxStepZ() - 1.0e-10);
      snapper.addSnapData(step3, new FootstepSnapData(snapTransform2));
      assertFalse(checker.isStepValid(step3, step1, step0));

      // good step
      DiscreteFootstep step4 = new DiscreteFootstep(0.0, 0.0, 0.0, RobotSide.RIGHT);
      RigidBodyTransform snapTransform3 = new RigidBodyTransform();
      snapper.addSnapData(step4, new FootstepSnapData(snapTransform3));
      assertTrue(checker.isStepValid(step4, step1, step0));
   }

   @Test
   public void testTooSmallFoothold()
   {
      DefaultFootstepPlannerParametersReadOnly parameters = new DefaultFootstepPlannerParameters();
      PlanarRegionFootstepSnapAndWiggler snapper = new TestSnapper();
      FootstepChecker checker = new FootstepChecker(parameters, footPolygons, snapper, null, registry);

      // add planar regions, otherwise will always be valid
      PlanarRegionsListGenerator planarRegionsListGenerator = new PlanarRegionsListGenerator();
      planarRegionsListGenerator.addRectangle(1.0, 1.0);
      PlanarRegionsList dummyRegions = planarRegionsListGenerator.getPlanarRegionsList();
      checker.setPlanarRegions(dummyRegions);

      double minFoothold = parameters.getMinFootholdPercent();

      DiscreteFootstep step0 = new DiscreteFootstep(0.0, 0.0, 0.0, RobotSide.LEFT);
      DiscreteFootstep step1 = new DiscreteFootstep(0.0, -0.2, 0.0, RobotSide.RIGHT);
      DiscreteFootstep step2 = new DiscreteFootstep(0.2, 0.0, 0.0, RobotSide.LEFT); // node to test
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
      snapper.addSnapData(step2, new FootstepSnapData(snapTransform0, goodFoothold));
      Assert.assertTrue(checker.isStepValid(step2, step1, step0));

      footholdArea = footArea * minFoothold * 0.99;
      side = Math.sqrt(footholdArea);
      ConvexPolygon2D badFoothold = new ConvexPolygon2D();
      badFoothold.addVertex(side / 2.0, side / 2.0);
      badFoothold.addVertex(-side / 2.0, side / 2.0);
      badFoothold.addVertex(side / 2.0, -side / 2.0);
      badFoothold.addVertex(-side / 2.0, -side / 2.0);
      badFoothold.update();

      // too small foothold
      snapper.addSnapData(step2, new FootstepSnapData(snapTransform0, badFoothold));
      Assert.assertFalse(checker.isStepValid(step2, step1, step0));
   }

   private static final double barelyTooSteepEpsilon = 1.0e-5;
   private static final int iters = 10000;

   @Test
   public void testSnappingToIncline()
   {
      DefaultFootstepPlannerParametersBasics parameters = new DefaultFootstepPlannerParameters()
      {
         // don't use 45
         @Override
         public double getMinSurfaceIncline()
         {
            return Math.toRadians(37.0);
         }

         @Override
         public boolean getWiggleWhilePlanning()
         {
            return true;
         }
      };

      PlanarRegionFootstepSnapAndWiggler snapper = new PlanarRegionFootstepSnapAndWiggler(footPolygons, parameters);
      testSnappingToInclinedPlane(parameters, snapper);
   }

   @Test
   public void testSnapAndWiggleOnIncline()
   {
      DefaultFootstepPlannerParametersBasics parameters = new DefaultFootstepPlannerParameters()
      {
         // don't use 45
         @Override
         public double getMinSurfaceIncline()
         {
            return Math.toRadians(37.0);
         }

         @Override
         public boolean getWiggleWhilePlanning()
         {
            return true;
         }
      };

      PlanarRegionFootstepSnapAndWiggler snapper = new PlanarRegionFootstepSnapAndWiggler(footPolygons, parameters);
      testSnappingToInclinedPlane(parameters, snapper);
   }

   public void testSnappingToInclinedPlane(DefaultFootstepPlannerParametersBasics parameters, PlanarRegionFootstepSnapAndWiggler snapper)
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

      FootstepChecker nodeChecker = new FootstepChecker(parameters, footPolygons, snapper, null, registry);
      nodeChecker.setPlanarRegions(planarRegionsList);

      DiscreteFootstep step0 = new DiscreteFootstep(-0.1, -0.1, 0.0, RobotSide.RIGHT); // the previous right foot position
      DiscreteFootstep step1 = new DiscreteFootstep(0.0, 0.1, 0.0, RobotSide.LEFT); // the left foot stance position for executing step 2
      DiscreteFootstep step2 = new DiscreteFootstep(0.0, -0.1, 0.0, RobotSide.RIGHT); // the candidate right foot position after the swing.

      // This should be true, as it just a translation of 10 cm forward.
      // TODO add getter for rejection reason or retreive from
      assertTrue(nodeChecker.isStepValid(step2, step1, step0));
//      assertEquals(null, nodeChecker.getRejectionReason());

      double rotationAngle;

      // test a bunch of independent roll/pitch valid angles
      for (rotationAngle = -parameters.getMinSurfaceIncline() + barelyTooSteepEpsilon; rotationAngle < parameters.getMinSurfaceIncline() - barelyTooSteepEpsilon; rotationAngle += 0.001)
      {
         transformToWorld.setIdentity();
         transformToWorld.appendRollRotation(rotationAngle);
         planarRegion.set(transformToWorld, polygons);
         nodeChecker.setPlanarRegions(planarRegionsList);
         snapper.setPlanarRegionsList(planarRegionsList);

         assertTrue(nodeChecker.isStepValid(step2, step1, step0));
         //         assertEquals(null, registry.getRejectionReason());

         transformToWorld.setIdentity();
         transformToWorld.appendPitchRotation(rotationAngle);
         planarRegion.set(transformToWorld, polygons);
         nodeChecker.setPlanarRegions(planarRegionsList);
         snapper.setPlanarRegionsList(planarRegionsList);
         snapper.clearSnapData();

         assertTrue(nodeChecker.isStepValid(step2, step1, step0));
//         assertEquals(null, registry.getRejectionReason());
      }

      // This should be false, as
      for (rotationAngle = parameters.getMinSurfaceIncline() + 0.001; rotationAngle < Math.toRadians(75); rotationAngle += 0.001)
      {
         transformToWorld.setIdentity();
         transformToWorld.appendRollRotation(rotationAngle);
         planarRegion.set(transformToWorld, polygons);
         snapper.setPlanarRegionsList(planarRegionsList);
         nodeChecker.setPlanarRegions(planarRegionsList);

         assertFalse(nodeChecker.isStepValid(step2, step1, step0), "rotation = " + rotationAngle);
         //         assertEquals("rotation = " + rotationAngle, BipedalFootstepPlannerNodeRejectionReason.SURFACE_NORMAL_TOO_STEEP_TO_SNAP, registry.getRejectionReason());

         transformToWorld.setIdentity();
         transformToWorld.appendPitchRotation(rotationAngle);
         planarRegion.set(transformToWorld, polygons);
         nodeChecker.setPlanarRegions(planarRegionsList);
         snapper.setPlanarRegionsList(planarRegionsList);

         assertFalse(nodeChecker.isStepValid(step2, step1, step0), "rotation = " + rotationAngle);
//         assertEquals("rotation = " + rotationAngle, BipedalFootstepPlannerNodeRejectionReason.SURFACE_NORMAL_TOO_STEEP_TO_SNAP, registry.getRejectionReason());
      }

      for (rotationAngle = -Math.toRadians(75); rotationAngle < -parameters.getMinSurfaceIncline(); rotationAngle += 0.001)
      {
         transformToWorld.setIdentity();
         transformToWorld.appendRollRotation(rotationAngle);
         planarRegion.set(transformToWorld, polygons);
         nodeChecker.setPlanarRegions(planarRegionsList);
         snapper.setPlanarRegionsList(planarRegionsList);

         assertFalse(nodeChecker.isStepValid(step2, step1, step0), "rotation = " + rotationAngle);
         //         assertEquals("rotation = " + rotationAngle, BipedalFootstepPlannerNodeRejectionReason.SURFACE_NORMAL_TOO_STEEP_TO_SNAP, registry.getRejectionReason());

         transformToWorld.setIdentity();
         transformToWorld.appendPitchRotation(rotationAngle);
         planarRegion.set(transformToWorld, polygons);
         nodeChecker.setPlanarRegions(planarRegionsList);
         snapper.setPlanarRegionsList(planarRegionsList);

         assertFalse(nodeChecker.isStepValid(step2, step1, step0), "rotation = " + rotationAngle);
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
         snapper.setPlanarRegionsList(planarRegionsList);

         Vector3D vertical = new Vector3D(0.0, 0.0, 1.0);
         Vector3D normal = new Vector3D(vertical);
         orientation3DReadOnly.transform(normal);
         //         QuaternionReadOnly noOrientation = new Quaternion(orientation3DReadOnly.getYaw(), 0.0, 0.0);

         double angleFromFlat = vertical.angle(normal);
         if (Math.abs(angleFromFlat) > parameters.getMinSurfaceIncline())
         {
            String message = "actual rotation = " + angleFromFlat + ", allowed rotation = " + parameters.getMinSurfaceIncline();
            assertFalse(nodeChecker.isStepValid(step2, step1, step0), message);
//            boolean correctRejection = BipedalFootstepPlannerNodeRejectionReason.SURFACE_NORMAL_TOO_STEEP_TO_SNAP == registry.getRejectionReason() ||
//                                       BipedalFootstepPlannerNodeRejectionReason.COULD_NOT_SNAP == registry.getRejectionReason();
//            assertTrue(message, correctRejection);
         }
         else
         {
            assertTrue(nodeChecker.isStepValid(step2, step1, step0));
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
      footstepPlannerParameters.setMaxXYWiggleDistance(maxXYWiggle);

      double maxYawWiggle = 0.7;
      footstepPlannerParameters.setMaxYawWiggle(maxYawWiggle);

      double footLength = 0.2;
      double footWidth = 0.1;
      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createFootPolygons(footLength, footWidth);

      PlanarRegionFootstepSnapAndWiggler snapper = new PlanarRegionFootstepSnapAndWiggler(footPolygons, footstepPlannerParameters);
      FootstepChecker checker = new FootstepChecker(footstepPlannerParameters, footPolygons, snapper, null, new YoRegistry("testRegistry"));

      DiscreteFootstep step0 = new DiscreteFootstep(0.0, -0.2, 0.0, RobotSide.RIGHT);
      DiscreteFootstep step1 = new DiscreteFootstep(0.0, 0.1, 0.0, RobotSide.LEFT);

      PlanarRegionsListGenerator planarRegionsListGenerator = new PlanarRegionsListGenerator();
      double regionEpsilon = 1e-5;

      // test just enough area
      double regionX = footLength + 2.0 * wiggleMinimum + regionEpsilon;
      double regionY = footWidth + 2.0 * wiggleMinimum + regionEpsilon;
      planarRegionsListGenerator.reset();

      planarRegionsListGenerator.translate(0.5 * maxXYWiggle, 0.5 * maxXYWiggle, 0.1);
      planarRegionsListGenerator.addRectangle(regionX, regionY);
      PlanarRegionsList regions = planarRegionsListGenerator.getPlanarRegionsList();
      snapper.setPlanarRegionsList(regions);
      checker.setPlanarRegions(regions);
      snapper.initialize();
      snapper.addSnapData(step0, new FootstepSnapData(new RigidBodyTransform()));
      boolean valid = checker.isStepValid(step1, step0, null);
      System.out.println(valid);

      // test just too little area along x
      snapper.reset();
      regionX = footLength + 2.0 * wiggleMinimum - regionEpsilon;
      regionY = footWidth + 2.0 * wiggleMinimum + regionEpsilon;
      planarRegionsListGenerator.reset();

      planarRegionsListGenerator.translate(0.5 * maxXYWiggle, 0.5 * maxXYWiggle, 0.1);
      planarRegionsListGenerator.addRectangle(regionX, regionY);
      regions = planarRegionsListGenerator.getPlanarRegionsList();
      snapper.setPlanarRegionsList(regions);
      checker.setPlanarRegions(regions);
      snapper.initialize();
      snapper.addSnapData(step0, new FootstepSnapData(new RigidBodyTransform()));
      valid = checker.isStepValid(step1, step0, null);
      System.out.println(valid);

      // test just too little area along y
      snapper.reset();
      regionX = footLength + 2.0 * wiggleMinimum + regionEpsilon;
      regionY = footWidth + 2.0 * wiggleMinimum - regionEpsilon;
      planarRegionsListGenerator.reset();

      planarRegionsListGenerator.translate(0.5 * maxXYWiggle, 0.5 * maxXYWiggle, 0.1);
      planarRegionsListGenerator.addRectangle(regionX, regionY);
      regions = planarRegionsListGenerator.getPlanarRegionsList();
      snapper.setPlanarRegionsList(regions);
      checker.setPlanarRegions(regions);
      snapper.initialize();
      snapper.addSnapData(step0, new FootstepSnapData(new RigidBodyTransform()));
      valid = checker.isStepValid(step1, step0, null);
      System.out.println(valid);
   }

   private class TestSnapper extends PlanarRegionFootstepSnapAndWiggler
   {
      public TestSnapper()
      {
         super(PlannerTools.createDefaultFootPolygons(), new DefaultFootstepPlannerParameters());
      }

      @Override
      protected FootstepSnapData computeSnapTransform(DiscreteFootstep footstepToSnap, DiscreteFootstep stanceStep)
      {
         throw new RuntimeException("In this test snapper add nodes manually.");
      }
   }
}
