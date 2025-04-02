package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerEnvironmentHandler;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticePoint;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.tools.PlanarRegionToHeightMapConverter;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.graphics.Graphics3DObjectTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class FootstepSnapAndWigglerTest
{
   private static boolean visualize = false;

   // For visualizable tests at the bottom
   private static final SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
   private SimulationConstructionSet scs;
   private YoGraphicsListRegistry graphicsListRegistry;
   private YoRegistry registry;
   private FootstepPlannerEnvironmentHandler environmentHandler;
   private FootstepSnapAndWiggler snapAndWiggler;
   private final DefaultFootstepPlannerParametersBasics parameters = new DefaultFootstepPlannerParameters();
   private YoDouble achievedDeltaInside;

   @BeforeEach
   public void setup()
   {
      visualize = visualize && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

      if (visualize)
      {
         scs = new SimulationConstructionSet(new Robot("testRobot"));
         achievedDeltaInside = new YoDouble("achievedDeltaInside", scs.getRootRegistry());
         scs.setGroundVisible(false);
         registry = new YoRegistry(getClass().getSimpleName());
         graphicsListRegistry = new YoGraphicsListRegistry();
         environmentHandler = new FootstepPlannerEnvironmentHandler();
         snapAndWiggler = new FootstepSnapAndWiggler(footPolygons, parameters, environmentHandler);
         graphicsListRegistry.addArtifactListsToPlotter(scs.createSimulationOverheadPlotterFactory().createOverheadPlotter().getPlotter());

         Graphics3DObject graphics3DObject = new Graphics3DObject();
         Graphics3DObjectTools.addPlanarRegionsList(graphics3DObject, DataSetIOTools.loadDataSet(DataSetName._20210419_111333_GPUCinders1).getPlanarRegionsList(), YoAppearance.DarkGray());
         scs.addStaticLinkGraphics(graphics3DObject);

         scs.addYoGraphicsListRegistry(graphicsListRegistry);
         scs.getRootRegistry().addChild(registry);
         scs.startOnAThread();
      }
      else
      {
         environmentHandler = new FootstepPlannerEnvironmentHandler();
         snapAndWiggler = new FootstepSnapAndWiggler(footPolygons, parameters, environmentHandler);
      }
   }

   @Test
   public void testTerriblePoint()
   {
      Random random = new Random(1738L);
      ConvexPolygon2D basePolygon = new ConvexPolygon2D();
      basePolygon.addVertex(-0.108, 0.048);
      basePolygon.addVertex(0.108, 0.030);
      basePolygon.addVertex(-0.108, -0.030);
      basePolygon.addVertex(-0.108, -0.048);
      basePolygon.update();

      double snapAreaResolution = 0.2;
      int iters = 1;
      for (int iter = 0; iter < iters; iter++)
      {
         // get a polygon transformed to a random location.
         RigidBodyTransform transform = new RigidBodyTransform();
         int gridXIndex = -50;
         int gridYIndex = -31;
         int yawIndex = 19;
         transform.getTranslation().set(LatticePoint.gridSizeXY * gridXIndex, LatticePoint.gridSizeXY * gridYIndex, 0.0);
         transform.getRotation().appendYawRotation(yawIndex * LatticePoint.gridSizeYaw);

         ConvexPolygon2D transformedPolygon = new ConvexPolygon2D(basePolygon);
         transformedPolygon.applyTransform(transform);

         // Here we want to collect all the points  under the foothold similar to what is done during snapping, but slightly different by assuming they're all
         // valid
         List<Point3D> footPointsInEnvironment = new ArrayList<>();
         Point2DReadOnly corner0 = transformedPolygon.getVertex(0);
         Point2DReadOnly corner1 = transformedPolygon.getVertex(1);
         Point2DReadOnly corner2 = transformedPolygon.getVertex(2);
         Point2DReadOnly corner3 = transformedPolygon.getVertex(3);

         Point2D pointOnEdge1 = new Point2D();
         Point2D pointOnEdge2 = new Point2D();
         Point2D footPointToSnap = new Point2D();

         double height = RandomNumbers.nextDouble(random, 1.0);

         for (double edgeAlpha = 0.0; edgeAlpha <= 1.0; edgeAlpha += snapAreaResolution)
         {
            pointOnEdge1.interpolate(corner0, corner1, edgeAlpha);
            pointOnEdge2.interpolate(corner3, corner2, edgeAlpha);

            for (double interiorAlpha = 0.0; interiorAlpha <= 1.0; interiorAlpha += snapAreaResolution)
            {
               footPointToSnap.interpolate(pointOnEdge1, pointOnEdge2, interiorAlpha);

               Point3D point = new Point3D(footPointToSnap.getX(), footPointToSnap.getY(), height);
               footPointsInEnvironment.add(point);
            }
         }


         // Now get the wrapped hull of this both before and after transform, and assert all the points are inside.
         ConvexPolygon2D croppedFoothold = new ConvexPolygon2D(Vertex3DSupplier.asVertex3DSupplier(footPointsInEnvironment));
         assertPointsInside(croppedFoothold, footPointsInEnvironment);

         List<Point2D> footPointsInFoot = footPointsInEnvironment.stream().map(point ->
                                                                               {
                                                                                  Point3D transformedPoint = new Point3D(point);
                                                                                  transformedPoint.applyInverseTransform(transform);
                                                                                  return new Point2D(transformedPoint);
                                                                               }).toList();


         // Transform crop, and make sure all the transformed points are inside
//         croppedFoothold.applyInverseTransform(transform, false);
//         assertPoint2DsInside(croppedFoothold, footPointsInFoot);


         // Create a polygon around all the transformed points, and make sure they're inside
         ConvexPolygon2D polygon2D = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(footPointsInFoot));
         assertPoint2DsInside(polygon2D, footPointsInFoot);
      }
   }


   @Test
   public void testTerriblePoints()
   {
      Random random = new Random(1738L);
      ConvexPolygon2D basePolygon = new ConvexPolygon2D();
      basePolygon.addVertex(-0.108, 0.048);
      basePolygon.addVertex(0.108, 0.030);
      basePolygon.addVertex(-0.108, -0.030);
      basePolygon.addVertex(-0.108, -0.048);
      basePolygon.update();

      double snapAreaResolution = 0.2;
      int iters = 100000;
      for (int iter = 0; iter < iters; iter++)
      {
         // get a polygon transformed to a random location.
         RigidBodyTransform transform = new RigidBodyTransform();
         int gridXIndex = RandomNumbers.nextInt(random, -100, 100);
         int gridYIndex = RandomNumbers.nextInt(random, -100, 100);
         int yawIndex = RandomNumbers.nextInt(random, 0, LatticePoint.yawDivisions);
         transform.getTranslation().set(LatticePoint.gridSizeXY * gridXIndex, LatticePoint.gridSizeXY * gridYIndex, 0.0);
         transform.getRotation().appendYawRotation(yawIndex * LatticePoint.gridSizeYaw);

         ConvexPolygon2D transformedPolygon = new ConvexPolygon2D(basePolygon);
         transformedPolygon.applyTransform(transform);

         // Here we want to collect all the points  under the foothold similar to what is done during snapping, but slightly different by assuming they're all
         // valid
         List<Point3D> footPointsInEnvironment = new ArrayList<>();
         Point2DReadOnly corner0 = transformedPolygon.getVertex(0);
         Point2DReadOnly corner1 = transformedPolygon.getVertex(1);
         Point2DReadOnly corner2 = transformedPolygon.getVertex(2);
         Point2DReadOnly corner3 = transformedPolygon.getVertex(3);

         Point2D pointOnEdge1 = new Point2D();
         Point2D pointOnEdge2 = new Point2D();
         Point2D footPointToSnap = new Point2D();

         double height = RandomNumbers.nextDouble(random, 1.0);

         for (double edgeAlpha = 0.0; edgeAlpha <= 1.0; edgeAlpha += snapAreaResolution)
         {
            pointOnEdge1.interpolate(corner0, corner1, edgeAlpha);
            pointOnEdge2.interpolate(corner3, corner2, edgeAlpha);

            for (double interiorAlpha = 0.0; interiorAlpha <= 1.0; interiorAlpha += snapAreaResolution)
            {
               footPointToSnap.interpolate(pointOnEdge1, pointOnEdge2, interiorAlpha);

               Point3D point = new Point3D(footPointToSnap.getX(), footPointToSnap.getY(), height);
               footPointsInEnvironment.add(point);
            }
         }

         List<Point2D> footPointsInFoot = footPointsInEnvironment.stream().map(point ->
                                                                               {
                                                                                  Point3D transformedPoint = new Point3D(point);
                                                                                  transformedPoint.applyInverseTransform(transform);
                                                                                  return new Point2D(transformedPoint);
                                                                               }).toList();

         // Now get the wrapped hull of this both before and after transform, and assert all the points are inside.
//         ConvexPolygon2D croppedFoothold = new ConvexPolygon2D(Vertex3DSupplier.asVertex3DSupplier(footPointsInEnvironment));
//         assertPointsInside(croppedFoothold, footPointsInEnvironment);

         // Transform crop, and make sure all the transformed points are inside
//         croppedFoothold.applyInverseTransform(transform, false);
//         assertPoint2DsInside(croppedFoothold, footPointsInFoot);


         // Create a polygon around all teh transformed points, and make sure they're inside
         ConvexPolygon2D polygon2D = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(footPointsInFoot));
         assertPoint2DsInside(polygon2D, footPointsInFoot);
      }
   }

   private static void assertPointsInside(ConvexPolygon2DReadOnly polygonToCheck, List<Point3D> points)
   {
      for (Point3D point : points)
      {
         double distance = polygonToCheck.signedDistance(new Point2D(point));
         assertTrue(polygonToCheck.isPointInside(new Point2D(point)), "Point " + point + " is not inside the polygon. Distance was " + distance);
         assertTrue(distance < 1e-3, "Point " + point + " is not inside the polygon. Distance was " + distance);
      }
   }

   private static void assertPoint2DsInside(ConvexPolygon2DReadOnly polygonToCheck, List<Point2D> points)
   {
      for (Point2D point : points)
      {
         double distance = polygonToCheck.signedDistance(point);
         assertTrue(polygonToCheck.isPointInside(point, 1e-3), "Point " + point + " is not inside the polygon. Distance was " + distance);
         assertTrue(distance < 1e-3, "Point " + point + " is not inside the polygon. Distance was " + distance);
      }
   }

   @Test
   public void testMaximumSnapHeightOnFlatRegions()
   {
      double groundHeight = -0.2;
      double maximumSnapHeight = 2.7;

      PlanarRegionsListGenerator planarRegionsListGenerator = new PlanarRegionsListGenerator();
      planarRegionsListGenerator.translate(0.0, 0.0, groundHeight);
      planarRegionsListGenerator.addRectangle(100.0, 100.0);

      // regions low enough to snap
      planarRegionsListGenerator.identity();
      double lowHeight0 = groundHeight + maximumSnapHeight - 1e-5;
      planarRegionsListGenerator.translate(1.0, -1.0, lowHeight0);
      planarRegionsListGenerator.addRectangle(1.0, 1.0);

      planarRegionsListGenerator.identity();
      double lowHeight1 = groundHeight + maximumSnapHeight - 0.5;
      planarRegionsListGenerator.translate(1.0, 0.0, lowHeight1);
      planarRegionsListGenerator.addRectangle(1.0, 1.0);

      planarRegionsListGenerator.identity();
      double lowHeight2 = groundHeight + 0.2;
      planarRegionsListGenerator.translate(1.0, 1.0, lowHeight2);
      planarRegionsListGenerator.addRectangle(1.0, 1.0);

      // regions too high to snap. However, this is a height map. Unlike the planar regions, there is no such thing as a multi-level approach. This means the
      // steps will be snapped to these heights.
      planarRegionsListGenerator.identity();
      double highHeight0 = groundHeight + maximumSnapHeight + 1e-5;
      planarRegionsListGenerator.translate(2.0, -1.0, highHeight0);
      planarRegionsListGenerator.addRectangle(1.0, 1.0);

      planarRegionsListGenerator.identity();
      double highHeight1 = groundHeight + maximumSnapHeight + 0.5;
      planarRegionsListGenerator.translate(2.0, 0.0, highHeight1);
      planarRegionsListGenerator.addRectangle(1.0, 1.0);

      planarRegionsListGenerator.identity();
      double highHeight2 = groundHeight + 100.0;
      planarRegionsListGenerator.translate(2.0, 1.0, highHeight2);
      planarRegionsListGenerator.addRectangle(1.0, 1.0);

      PlanarRegionsList planarRegionsList = planarRegionsListGenerator.getPlanarRegionsList();
      DefaultFootstepPlannerParameters footstepPlannerParameters = new DefaultFootstepPlannerParameters();
      footstepPlannerParameters.setMaximumSnapHeight(maximumSnapHeight);
      FootstepPlannerEnvironmentHandler environmentHandler = new FootstepPlannerEnvironmentHandler();
      FootstepSnapAndWiggler snapper  = new FootstepSnapAndWiggler(PlannerTools.createDefaultFootPolygons(), footstepPlannerParameters, environmentHandler);

      HeightMapMessage heightMapMessage = PlanarRegionToHeightMapConverter.convertFromPlanarRegionsToHeightMap(planarRegionsList);
      environmentHandler.setHeightMap(HeightMapMessageTools.unpackMessage(heightMapMessage));

      RigidBodyTransform expectedTransform = new RigidBodyTransform();
      double epsilon = 1e-5;

      DiscreteFootstep stanceNode = new DiscreteFootstep(0.0, 0.0);
      snapper.snapFootstep(stanceNode, null, false);

      // test regions low enough to snap
      // the high height doesn't get filtered in the height map.
      FootstepSnapData snapData = snapper.snapFootstep(new DiscreteFootstep(1.0, -1.0), stanceNode, false);
      expectedTransform.setTranslationAndIdentityRotation(new Vector3D(0.0, 0.0, lowHeight0));
      EuclidCoreTestTools.assertEquals(expectedTransform, snapData.getSnapTransform(), epsilon);

      snapData = snapper.snapFootstep(new DiscreteFootstep(1.0, 0.0), stanceNode, false);
      expectedTransform.setTranslationAndIdentityRotation(new Vector3D(0.0, 0.0, lowHeight1));
      EuclidCoreTestTools.assertEquals(expectedTransform, snapData.getSnapTransform(), epsilon);

      snapData = snapper.snapFootstep(new DiscreteFootstep(1.0, 1.0), stanceNode, false);
      expectedTransform.setTranslationAndIdentityRotation(new Vector3D(0.0, 0.0, lowHeight2));
      EuclidCoreTestTools.assertEquals(expectedTransform, snapData.getSnapTransform(), epsilon);

      // test regions high enough to snap. Unlike with the planar regions, these high heights won't be filtered out, because the height map doesn't have a
      // concept of "multi-level".
      snapData = snapper.snapFootstep(new DiscreteFootstep(2.0, -1.0), stanceNode, false);
      expectedTransform.setTranslationAndIdentityRotation(new Vector3D(0.0, 0.0, highHeight0));
      EuclidCoreTestTools.assertEquals(expectedTransform, snapData.getSnapTransform(), epsilon);

      snapData = snapper.snapFootstep(new DiscreteFootstep(2.0, 0.0), stanceNode, false);
      expectedTransform.setTranslationAndIdentityRotation(new Vector3D(0.0, 0.0, highHeight1));
      EuclidCoreTestTools.assertEquals(expectedTransform, snapData.getSnapTransform(), epsilon);

      snapData = snapper.snapFootstep(new DiscreteFootstep(2.0, 1.0), stanceNode, false);
      expectedTransform.setTranslationAndIdentityRotation(new Vector3D(0.0, 0.0, highHeight2));
      EuclidCoreTestTools.assertEquals(expectedTransform, snapData.getSnapTransform(), epsilon);
   }

   @Test
   public void testMaximumSnapHeightOnSlopedRegion()
   {
      double groundHeight = -0.2;
      double maximumSnapHeight = 2.7;
      double rotatedAngle = Math.toRadians(- 45.0);

      PlanarRegionsListGenerator planarRegionsListGenerator = new PlanarRegionsListGenerator();
      planarRegionsListGenerator.translate(0.0, 0.0, groundHeight);
      planarRegionsListGenerator.addRectangle(100.0, 100.0);
      planarRegionsListGenerator.rotate(rotatedAngle, Axis3D.Y);
      planarRegionsListGenerator.addRectangle(100.0, 100.0);

      PlanarRegionsList planarRegionsList = planarRegionsListGenerator.getPlanarRegionsList();
      DefaultFootstepPlannerParameters footstepPlannerParameters = new DefaultFootstepPlannerParameters();
      footstepPlannerParameters.setMaximumSnapHeight(maximumSnapHeight);
      FootstepPlannerEnvironmentHandler environmentHandler = new FootstepPlannerEnvironmentHandler();
      FootstepSnapAndWiggler snapper  = new FootstepSnapAndWiggler(PlannerTools.createDefaultFootPolygons(), footstepPlannerParameters, environmentHandler);

      HeightMapMessage heightMapMessage = PlanarRegionToHeightMapConverter.convertFromPlanarRegionsToHeightMap(planarRegionsList);
      environmentHandler.setHeightMap(HeightMapMessageTools.unpackMessage(heightMapMessage));

      RigidBodyTransform expectedTransform = new RigidBodyTransform();
      double epsilon = 1e-5;

      DiscreteFootstep stanceNode = new DiscreteFootstep(0.0, 0.0);
      snapper.snapFootstep(stanceNode, null, false);

      FootstepSnapData snapData = snapper.snapFootstep(new DiscreteFootstep(- 1.0, 0.0), stanceNode, false);
      expectedTransform.setTranslationAndIdentityRotation(new Vector3D(0.0, 0.0, groundHeight));
      EuclidCoreTestTools.assertEquals(expectedTransform, snapData.getSnapTransform(), epsilon);

      snapData = snapper.snapFootstep(new DiscreteFootstep(1.0, 0.0), stanceNode, false);
      assertEquals(rotatedAngle, snapData.getSnapTransform().getRotation().getPitch(), epsilon);

      snapData = snapper.snapFootstep(new DiscreteFootstep(3.0, 0.0), stanceNode, false);
      assertEquals(rotatedAngle, snapData.getSnapTransform().getRotation().getPitch(), epsilon);

      snapData = snapper.snapFootstep(new DiscreteFootstep(-3.0, 0.0), stanceNode, false);
      assertEquals(0.0, snapData.getSnapTransform().getRotation().getPitch(), epsilon);
   }

   @Test
   public void testOverlapDetection()
   {
      double epsilon = 1e-6;

      ConvexPolygon2D unitSquare = new ConvexPolygon2D();
      unitSquare.addVertex(1.0, 1.0);
      unitSquare.addVertex(1.0, -1.0);
      unitSquare.addVertex(-1.0, 1.0);
      unitSquare.addVertex(-1.0, -1.0);
      unitSquare.update();
      SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>(() -> new ConvexPolygon2D(unitSquare));

      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      parameters.setMinClearanceFromStance(0.0);

      FootstepPlannerEnvironmentHandler environmentHandler = new FootstepPlannerEnvironmentHandler();
      FootstepSnapAndWiggler snapper = new FootstepSnapAndWiggler(footPolygons, parameters, environmentHandler);

      FootstepSnapData snapData1 = new FootstepSnapData();
      FootstepSnapData snapData2 = new FootstepSnapData();

      DiscreteFootstep node1 = new DiscreteFootstep(0.0, 0.0, 0.0, RobotSide.LEFT);
      DiscreteFootstep node2 = new DiscreteFootstep(0.0, 0.0, 0.0, RobotSide.RIGHT);

      snapData1.getSnapTransform().setIdentity();
      snapData2.getSnapTransform().setIdentity();
      snapData1.getWiggleTransformInWorld().setIdentity();
      snapData2.getWiggleTransformInWorld().setIdentity();

      boolean overlap = snapper.stepsAreTooClose(node1, snapData1, node2, snapData2);
      Assertions.assertTrue(overlap);

      snapData2.getSnapTransform().getTranslation().set(2.0 - epsilon, 0.0, 0.0);
      overlap = snapper.stepsAreTooClose(node1, snapData1, node2, snapData2);
      Assertions.assertTrue(overlap);

      snapData2.getSnapTransform().getTranslation().set(2.0 + epsilon, 0.0, 0.0);
      overlap = snapper.stepsAreTooClose(node1, snapData1, node2, snapData2);
      Assertions.assertFalse(overlap);

      snapData1.getSnapTransform().getRotation().setYawPitchRoll(0.0, 0.0, Math.toRadians(45.0));
      snapData2.getSnapTransform().getTranslation().set(0.0, 1.0 + Math.sqrt(0.5) - epsilon, 0.0);
      overlap = snapper.stepsAreTooClose(node1, snapData1, node2, snapData2);
      Assertions.assertTrue(overlap);

      snapData2.getSnapTransform().getTranslation().set(0.0, 1.0 + Math.sqrt(0.5) + epsilon, 0.0);
      overlap = snapper.stepsAreTooClose(node1, snapData1, node2, snapData2);
      Assertions.assertFalse(overlap);
   }


   @Test
   public void testSnappingToFlatGroundHeight()
   {
      double flatGroundHeight = 0.7;
      snapAndWiggler.setFlatGroundHeight(flatGroundHeight);
      environmentHandler.setHeightMap(null);

      DiscreteFootstep footstep = new DiscreteFootstep(3, -2, 5, RobotSide.LEFT);
      FootstepSnapData snapData = snapAndWiggler.snapFootstep(footstep);
      RigidBodyTransform snappedStepTransform = snapData.getSnappedStepTransform(footstep);
      double epsilon = 1e-7;
      assertEquals(snappedStepTransform.getTranslation().getZ(), flatGroundHeight, epsilon, "Flat ground snap height is not equal");
   }

   public void testStanceFootClearance()
   {
      DataSet dataset = DataSetIOTools.loadDataSet(DataSetName._20210419_111333_GPUCinders1);
      PlanarRegionsList planarRegionsList = dataset.getPlanarRegionsList();

      parameters.setEnableConcaveHullWiggler(true);
      parameters.setWiggleInsideDeltaTarget(0.07);
      parameters.setWiggleInsideDeltaMinimum(0.02);
      parameters.setMaxXYWiggleDistance(0.2);
      parameters.setMinClearanceFromStance(0.03);

      snapAndWiggler.initialize();

      HeightMapMessage heightMapMessage = PlanarRegionToHeightMapConverter.convertFromPlanarRegionsToHeightMap(planarRegionsList);
      environmentHandler.setHeightMap(HeightMapMessageTools.unpackMessage(heightMapMessage));

      DiscreteFootstep stanceStep = new DiscreteFootstep(105, 82, 3, RobotSide.LEFT);
      DiscreteFootstep candidateStep = new DiscreteFootstep(109, 80, 2, RobotSide.RIGHT);

      RigidBodyTransform stanceSnapTransform = new RigidBodyTransform(new Quaternion(-0.00521871,0.01066136,-0.00008137,0.99992954), new Vector3D(-0.00110121,0.00147726,0.88437723));
      FootstepSnapData stanceSnapData = new FootstepSnapData(stanceSnapTransform);
      stanceSnapData.setRegionIndex(2);
      snapAndWiggler.addSnapData(stanceStep, stanceSnapData);

      if (visualize)
      {
         FootstepSnapData snapWiggleData = snapAndWiggler.snapFootstep(candidateStep, stanceStep, true);
         achievedDeltaInside.set(snapWiggleData.getAchievedInsideDelta());
         scs.tickAndUpdate();

         scs.cropBuffer();
         ThreadTools.sleepForever();
      }
   }

   public static void main(String[] args)
   {
      FootstepSnapAndWigglerTest footstepSnapAndWigglerTest = new FootstepSnapAndWigglerTest();
      visualize = true;
      footstepSnapAndWigglerTest.setup();
      footstepSnapAndWigglerTest.testStanceFootClearance();
   }
}
