package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import perception_msgs.msg.dds.TerrainMapMessage;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.graphSearch.EnvironmentHandler;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
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
import us.ihmc.perception.gpuMapping.TerrainMapData;
import us.ihmc.perception.gpuMapping.TerrainMapMessageTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.graphics.Graphics3DObjectTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import static org.junit.jupiter.api.Assertions.*;

public class FootstepSnapAndWigglerTest
{
   private static boolean visualize = false;

   // For visualizable tests at the bottom
   private static final SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
   private SimulationConstructionSet scs;
   private YoGraphicsListRegistry graphicsListRegistry;
   private YoRegistry registry;
   private EnvironmentHandler environmentHandler;
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
         environmentHandler = new EnvironmentHandler();
         snapAndWiggler = new FootstepSnapAndWiggler(footPolygons, parameters, environmentHandler);
         graphicsListRegistry.addArtifactListsToPlotter(scs.createSimulationOverheadPlotterFactory().createOverheadPlotter().getPlotter());

         Graphics3DObject graphics3DObject = new Graphics3DObject();
         Graphics3DObjectTools.addPlanarRegionsList(graphics3DObject,
                                                    DataSetIOTools.loadDataSet(DataSetName._20210419_111333_GPUCinders1).getPlanarRegionsList(),
                                                    YoAppearance.DarkGray());
         scs.addStaticLinkGraphics(graphics3DObject);

         scs.addYoGraphicsListRegistry(graphicsListRegistry);
         scs.getRootRegistry().addChild(registry);
         scs.startOnAThread();
      }
      else
      {
         environmentHandler = new EnvironmentHandler();
         snapAndWiggler = new FootstepSnapAndWiggler(footPolygons, parameters, environmentHandler);
      }
   }

   @Test
   public void testMaximumSnapHeightOnFlatRegions()
   {
      double groundHeight = -0.2;
      double maximumSnapHeight = 2.7;

      PlanarRegionsListGenerator planarRegionsListGenerator = new PlanarRegionsListGenerator();
      planarRegionsListGenerator.translate(0.0, 0.0, groundHeight);
      planarRegionsListGenerator.addRectangle(10.0, 10.0);

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
      EnvironmentHandler environmentHandler = new EnvironmentHandler();
      FootstepSnapAndWiggler snapper = new FootstepSnapAndWiggler(PlannerTools.createDefaultFootPolygons(), footstepPlannerParameters, environmentHandler);

      TerrainMapMessage heightMapMessage = PlanarRegionToHeightMapConverter.convertFromPlanarRegionsToHeightMap(planarRegionsList);
      environmentHandler.setTerrainMapData(TerrainMapMessageTools.unpackMessage(heightMapMessage));

      RigidBodyTransform expectedTransform = new RigidBodyTransform();
      double epsilon = 1e-2;

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
      double rotatedAngle = Math.toRadians(-45.0);

      PlanarRegionsListGenerator planarRegionsListGenerator = new PlanarRegionsListGenerator();
      planarRegionsListGenerator.translate(0.0, 0.0, groundHeight);
      planarRegionsListGenerator.addRectangle(10.0, 00.0);
      planarRegionsListGenerator.rotate(rotatedAngle, Axis3D.Y);
      planarRegionsListGenerator.addRectangle(10.0, 10.0);

      PlanarRegionsList planarRegionsList = planarRegionsListGenerator.getPlanarRegionsList();
      DefaultFootstepPlannerParameters footstepPlannerParameters = new DefaultFootstepPlannerParameters();
      footstepPlannerParameters.setMaximumSnapHeight(maximumSnapHeight);
      EnvironmentHandler environmentHandler = new EnvironmentHandler();
      FootstepSnapAndWiggler snapper = new FootstepSnapAndWiggler(PlannerTools.createDefaultFootPolygons(), footstepPlannerParameters, environmentHandler);

      TerrainMapMessage terrainMapMessage = PlanarRegionToHeightMapConverter.convertFromPlanarRegionsToHeightMap(planarRegionsList);
      environmentHandler.setTerrainMapData(TerrainMapMessageTools.unpackMessage(terrainMapMessage));
      RigidBodyTransform expectedTransform = new RigidBodyTransform();
      double epsilon = 1e-2;

      DiscreteFootstep stanceNode = new DiscreteFootstep(0.0, 0.0);
      snapper.snapFootstep(stanceNode, null, false);

      FootstepSnapData snapData = snapper.snapFootstep(new DiscreteFootstep(-1.0, 0.0), stanceNode, false);
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

      EnvironmentHandler environmentHandler = new EnvironmentHandler();
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

      TerrainMapMessage terrainMapMessage = PlanarRegionToHeightMapConverter.convertFromPlanarRegionsToHeightMap(planarRegionsList);

      DiscreteFootstep stanceStep = new DiscreteFootstep(105, 82, 3, RobotSide.LEFT);
      DiscreteFootstep candidateStep = new DiscreteFootstep(109, 80, 2, RobotSide.RIGHT);

      RigidBodyTransform stanceSnapTransform = new RigidBodyTransform(new Quaternion(-0.00521871, 0.01066136, -0.00008137, 0.99992954),
                                                                      new Vector3D(-0.00110121, 0.00147726, 0.88437723));
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
