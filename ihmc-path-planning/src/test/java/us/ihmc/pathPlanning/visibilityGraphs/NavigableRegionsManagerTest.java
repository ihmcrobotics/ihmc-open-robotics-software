package us.ihmc.pathPlanning.visibilityGraphs;

import org.junit.jupiter.api.*;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.shape.primitives.Ellipsoid3D;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanner;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMapWithNavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.NavigableExtrusionDistanceCalculator;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.PlanarRegionFilter;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.ObstacleAndCliffAvoidanceProcessor;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.PathOrientationCalculator;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityTools;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullDecomposition;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.testing.JUnitTools;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

public class NavigableRegionsManagerTest
{
   private static boolean visualize = false;
   private static final double epsilon = 1e-4;
   private static final double proximityEpsilon = 2e-2;

   // For enabling helpful prints.
   private static boolean DEBUG = false;

   private static VisibilityGraphsTestVisualizerApplication visualizerApplication = null;
   // Because we use JavaFX, there will be two instance of VisibilityGraphsFrameworkTest, one for running the test and one starting the ui. The messager has to be static so both the ui and test use the same instance.
   private static JavaFXMessager messager = null;

   // The following are used for collision checks.
   private static final double walkerOffsetHeight = 0.75;

   private static final double obstacleExtrusionDistance = 0.2;
   private static final double preferredObstacleExtrusionDistance = 1.0;
   private static final Vector3D walkerRadii = new Vector3D(obstacleExtrusionDistance, preferredObstacleExtrusionDistance, 0.5);
   private static final Vector3D walkerBox = new Vector3D(2.0 * obstacleExtrusionDistance, 2.0 * preferredObstacleExtrusionDistance, 1.0);


   @BeforeEach
   public void setup()
   {
      visualize = visualize && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
      DEBUG = (visualize || (DEBUG && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer()));

      if (visualize)
      {
         visualizerApplication = new VisibilityGraphsTestVisualizerApplication();
         visualizerApplication.startOnAThread();

         messager = visualizerApplication.getMessager();

         messager.submitMessage(UIVisibilityGraphsTopics.EnableWalkerAnimation, false);
         messager.submitMessage(UIVisibilityGraphsTopics.WalkerOffsetHeight, walkerOffsetHeight);
         messager.submitMessage(UIVisibilityGraphsTopics.WalkerSize, walkerRadii);
//         messager.submitMessage(UIVisibilityGraphsTopics.WalkerBoxSize, walkerBox);
      }
   }

   @AfterEach
   public void tearDown() throws Exception
   {
      if (visualize)
      {
         visualizerApplication.stop();
         visualizerApplication = null;
         messager = null;
      }
   }

   @Test
   public void testFlatGroundWithWallInlineWithWall()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(createFlatGroundWithWallEnvironment());

      // test aligned with the edge of the wall, requiring slight offset
      Point3D start = new Point3D(-15.0, 0.0, 0.0);
      Point3D goal = new Point3D(-5.0, 0.0, 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      ObstacleAndCliffAvoidanceProcessor postProcessor = new ObstacleAndCliffAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator.computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testFlatGroundWithWallOnOppositeSidesOfWall()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(createFlatGroundWithWallEnvironment());

      // test on opposite sides of the wall, requiring going around it
      Point3D start = new Point3D(-15.0, 1.0, 0.0);
      Point3D goal = new Point3D(-5.0, 1.0, 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      ObstacleAndCliffAvoidanceProcessor postProcessor = new ObstacleAndCliffAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator.computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testFlatGroundWithWallStraightShotButVeryNearWall()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(createFlatGroundWithWallEnvironment());

      // test on opposite sides of the wall, requiring going around it
      Point3D start = new Point3D(-15.0, -0.05 * parameters.getPreferredObstacleExtrusionDistance(), 0.0);
      Point3D goal = new Point3D(-5.0, -0.05 * parameters.getPreferredObstacleExtrusionDistance(), 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      ObstacleAndCliffAvoidanceProcessor postProcessor = new ObstacleAndCliffAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator.computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());

   }

   @Test
   public void testFlatGroundWithWallStraightShotButNearWall()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(createFlatGroundWithWallEnvironment());

      // test on opposite sides of the wall, requiring going around it
      Point3D start = new Point3D(-15.0, -0.1 * parameters.getPreferredObstacleExtrusionDistance(), 0.0);
      Point3D goal = new Point3D(-5.0, -0.1 * parameters.getPreferredObstacleExtrusionDistance(), 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      ObstacleAndCliffAvoidanceProcessor postProcessor = new ObstacleAndCliffAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator.computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testFlatGroundWithWallAlmostStraightShot()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(createFlatGroundWithWallEnvironment());

      // test on opposite sides of the wall, requiring going around it
      Point3D start = new Point3D(-15.0, -0.95 * parameters.getPreferredObstacleExtrusionDistance(), 0.0);
      Point3D goal = new Point3D(-5.0, -0.95 * parameters.getPreferredObstacleExtrusionDistance(), 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      ObstacleAndCliffAvoidanceProcessor postProcessor = new ObstacleAndCliffAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator.computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testFlatGroundWithWallStraightShot()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(createFlatGroundWithWallEnvironment());

      // test on opposite sides of the wall, requiring going around it
      Point3D start = new Point3D(-15.0, -1.05 * parameters.getPreferredObstacleExtrusionDistance(), 0.0);
      Point3D goal = new Point3D(-5.0, -1.05 * parameters.getPreferredObstacleExtrusionDistance(), 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      ObstacleAndCliffAvoidanceProcessor postProcessor = new ObstacleAndCliffAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator.computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testFlatGroundWithBoxInlineWithWall()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(createFlatGroundWithBoxEnvironment());

      // test aligned with the edge of the wall, requiring slight offset
      Point3D start = new Point3D(-15.0, 0.0, 0.0);
      Point3D goal = new Point3D(-5.0, 0.0, 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      ObstacleAndCliffAvoidanceProcessor postProcessor = new ObstacleAndCliffAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator.computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testFlatGroundWithBoxOnOppositeSidesOfWall()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(createFlatGroundWithBoxEnvironment());

      // test on opposite sides of the wall, requiring going around it
      Point3D start = new Point3D(-15.0, 1.0, 0.0);
      Point3D goal = new Point3D(-5.0, 1.0, 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      ObstacleAndCliffAvoidanceProcessor postProcessor = new ObstacleAndCliffAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator.computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testFlatGroundWithBoxStraightShotButVeryNearWall()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(createFlatGroundWithBoxEnvironment());

      // test on opposite sides of the wall, requiring going around it
      Point3D start = new Point3D(-15.0, -0.05 * parameters.getPreferredObstacleExtrusionDistance(), 0.0);
      Point3D goal = new Point3D(-5.0, -0.05 * parameters.getPreferredObstacleExtrusionDistance(), 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      ObstacleAndCliffAvoidanceProcessor postProcessor = new ObstacleAndCliffAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator.computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testFlatGroundWithBoxStraightShotButNearWall()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(createFlatGroundWithBoxEnvironment());

      // test on opposite sides of the wall, requiring going around it
      Point3D start = new Point3D(-15.0, -0.1 * parameters.getPreferredObstacleExtrusionDistance(), 0.0);
      Point3D goal = new Point3D(-5.0, -0.1 * parameters.getPreferredObstacleExtrusionDistance(), 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      ObstacleAndCliffAvoidanceProcessor postProcessor = new ObstacleAndCliffAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator.computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testFlatGroundWithBoxAlmostStraightShot()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(createFlatGroundWithBoxEnvironment());

      // test on opposite sides of the wall, requiring going around it
      Point3D start = new Point3D(-15.0, -0.95 * parameters.getPreferredObstacleExtrusionDistance(), 0.0);
      Point3D goal = new Point3D(-5.0, -0.95 * parameters.getPreferredObstacleExtrusionDistance(), 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      ObstacleAndCliffAvoidanceProcessor postProcessor = new ObstacleAndCliffAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator.computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testFlatGroundWithBoxStraightShot()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(createFlatGroundWithBoxEnvironment());

      // test on opposite sides of the wall, requiring going around it
      Point3D start = new Point3D(-15.0, -1.05 * parameters.getPreferredObstacleExtrusionDistance(), 0.0);
      Point3D goal = new Point3D(-5.0, -1.05 * parameters.getPreferredObstacleExtrusionDistance(), 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      ObstacleAndCliffAvoidanceProcessor postProcessor = new ObstacleAndCliffAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator.computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testFlatGroundWithTwoDifferentWalls()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(createFlatGroundTwoDifferentWidthWallsEnvironment());

      // test straight shot, initially going to one of the nodes
      Point3D start = new Point3D(-15.0, -0.5 * parameters.getObstacleExtrusionDistance(), 0.0);
      Point3D goal = new Point3D(-5.0, -0.5 * parameters.getObstacleExtrusionDistance(), 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      ObstacleAndCliffAvoidanceProcessor postProcessor = new ObstacleAndCliffAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator.computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution());

      /*
      if (visualize)
      {
         visualize(path, planarRegionsList, start, goal);
      }
      */

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());

      start = new Point3D(-15.0, 1.0 * parameters.getPreferredObstacleExtrusionDistance(), 0.0);

      path = navigableRegionsManager.calculateBodyPath(start, goal);
      posePath = orientationCalculator.computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testFlatGroundBetweenWallOpening()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(createFlatGroundWithWallOpeningEnvironment());

      // test aligned with the edge of the wall, requiring slight offset
      Point3D start = new Point3D(-15.0, 1.5, 0.0);
      Point3D goal = new Point3D(-5.0, 1.5, 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      ObstacleAndCliffAvoidanceProcessor postProcessor = new ObstacleAndCliffAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator.computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testFlatGroundBetweenWallOpeningStraightShot()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(createFlatGroundWithWallOpeningEnvironment());

      // test aligned with the edge of the wall, requiring slight offset
      Point3D start = new Point3D(-15.0, 0.0, 0.0);
      Point3D goal = new Point3D(-5.0, 0.0, 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);

      ObstacleAndCliffAvoidanceProcessor postProcessor = new ObstacleAndCliffAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator.computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());

   }

   @Test
   public void testFlatGroundBetweenAwkwardWallOpening()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(createFlatGroundWithWallAwkwardOpeningEnvironment());

      // test aligned with the edge of the wall, requiring slight offset
      Point3D start = new Point3D(-15.0, 1.5, 0.0);
      Point3D goal = new Point3D(-5.0, 1.5, 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), new ObstacleAndCliffAvoidanceProcessor(parameters));
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator.computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testFlatGroundBetweenBoxesOpening()
   {
      /*
      This test sets up an environment where the first, aggressive pass will hug the left of the opening. It is then further away from the other side than
      the preferred distance, so it may not queue that up as a distance it should consider.
       */
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(createFlatGroundWithBoxesEnvironment());

      // test aligned with the edge of the wall, requiring slight offset
      Point3D start = new Point3D(-15.0, 1.5, 0.0);
      Point3D goal = new Point3D(-5.0, 1.5, 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), new ObstacleAndCliffAvoidanceProcessor(parameters));
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator.computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testFlatGroundBetweenBoxesOpeningStraightShot()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(createFlatGroundWithBoxesEnvironment());

      // test aligned with the edge of the wall, requiring slight offset
      Point3D start = new Point3D(-15.0, 0.0, 0.0);
      Point3D goal = new Point3D(-5.0, 0.0, 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      ObstacleAndCliffAvoidanceProcessor postProcessor = new ObstacleAndCliffAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator.computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution());

      if (visualize)
      {
         visualize(posePath, parameters,  planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testFlatGroundBetweenBoxInMiddle()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(createFlatGroundWithBoxInMiddleEnvironment());

      // test aligned with the edge of the wall, requiring slight offset
      Point3D start = new Point3D(-15.0, 0.0, 0.0);
      Point3D goal = new Point3D(-5.0, 0.0, 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), new ObstacleAndCliffAvoidanceProcessor(parameters));
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator.computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testFlatGroundBetweenBoxInMiddleButExtraSupportToTheLeft()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(createFlatGroundWithBoxInMiddleEnvironmentButIslandToTheLeft());

      // test aligned with the edge of the wall, requiring slight offset
      Point3D start = new Point3D(-15.0, 0.0, 0.0);
      Point3D goal = new Point3D(-5.0, 0.0, 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), new ObstacleAndCliffAvoidanceProcessor(parameters));
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator.computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }


   private static void checkPath(List<? extends Pose3DReadOnly> path, Point3DReadOnly start, Point3DReadOnly goal, VisibilityGraphsParametersReadOnly parameters,
                                 PlanarRegionsList planarRegionsList, List<VisibilityMapWithNavigableRegion> navigableRegionsList)
   {
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), null);

      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());
      List<Point3DReadOnly> originalPath = navigableRegionsManager.calculateBodyPath(start, goal);

      String errorMessages = "";


      int numberOfPoints = path.size();
//      if (numberOfPoints < originalPath.size())
//         errorMessages += fail("number of points is not what was expected.");

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(start, path.get(0).getPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(goal, path.get(numberOfPoints - 1).getPosition(), epsilon);

      for (Pose3DReadOnly point : path)
      {
         assertFalse(point.containsNaN());
      }

      WaypointDefinedBodyPathPlanner calculatedPath = new WaypointDefinedBodyPathPlanner();
      calculatedPath.setPoseWaypoints(path);

      WaypointDefinedBodyPathPlanner expectedPathNoAvoidance = new WaypointDefinedBodyPathPlanner();
      expectedPathNoAvoidance.setWaypoints(originalPath);

      double distanceAlongExpectedPath = 0.0;

      Ellipsoid3D walkerShape = new Ellipsoid3D();
      walkerShape.setRadii(walkerRadii);
      List<Point3D> collisions = new ArrayList<>();


      for (double alpha = 0.05; alpha < 1.0; alpha += 0.001)
      {
         Pose2D expectedPose = new Pose2D();
         Pose2D actualPose = new Pose2D();

         expectedPathNoAvoidance.getPointAlongPath(alpha, expectedPose);
         calculatedPath.getPointAlongPath(alpha, actualPose);

         // assert it doesn't deviate too much
         if (!expectedPose.getPosition().geometricallyEquals(actualPose.getPosition(), preferredObstacleExtrusionDistance))
            errorMessages += fail("Pose was not as expected at alpha = " + alpha);

         Point3DReadOnly position3D = PlanarRegionTools.projectPointToPlanesVertically(new Point3D(actualPose.getPosition()), planarRegionsList);
         Quaternion orientation = new Quaternion(actualPose.getYaw(), 0.0, 0.0);

         if (position3D == null)
            position3D = new Point3D(actualPose.getPosition());

         if (visualize)
         {

            messager.submitMessage(UIVisibilityGraphsTopics.WalkerPosition, new Point3D(position3D));
            messager.submitMessage(UIVisibilityGraphsTopics.WalkerOrientation, orientation);

            ThreadTools.sleep(10);
         }

         // check that it's always moving along
         Point2D pointAlongExpectedPath = new Point2D();
         double newDistanceAlongExpectedPath = expectedPathNoAvoidance.getClosestPoint(pointAlongExpectedPath, actualPose);
         if (newDistanceAlongExpectedPath < distanceAlongExpectedPath)
            errorMessages += fail("Not moving forward.");
         distanceAlongExpectedPath = newDistanceAlongExpectedPath;

         // check that it doesn't get too close to an obstacle
         double distanceToObstacles = Double.MAX_VALUE;
         Point2D closestPointOverall = new Point2D();
         for (VisibilityMapWithNavigableRegion navigableRegion : navigableRegionsList)
         {
            for (Cluster obstacleCluster : navigableRegion.getObstacleClusters())
            {
               List<Point2DReadOnly> clusterPolygon = obstacleCluster.getNonNavigableExtrusionsInWorld2D();
               Point2D closestPointOnCluster = new Point2D();
               double distanceToCluster = VisibilityTools.distanceToCluster(actualPose.getPosition(), clusterPolygon, closestPointOnCluster, null);
               if (distanceToCluster < distanceToObstacles)
               {
                  distanceToObstacles = distanceToCluster;
                  closestPointOverall = closestPointOnCluster;
               }
            }
         }

         // check the walker position
         Point3D walkerBody3D = new Point3D(position3D);
         walkerBody3D.addZ(walkerOffsetHeight);
         walkerShape.getPosition().set(walkerBody3D);
         walkerShape.getOrientation().set(orientation);

         String newErrorMessages = walkerCollisionChecks(walkerShape, planarRegionsList, collisions);
         errorMessages += newErrorMessages;
         if (!newErrorMessages.isEmpty() && visualize)
            messager.submitMessage(UIVisibilityGraphsTopics.WalkerCollisionLocations, collisions);


         distanceToObstacles += parameters.getObstacleExtrusionDistance();

         if (visualize && distanceToObstacles < preferredObstacleExtrusionDistance + proximityEpsilon)
         {
            Point3DReadOnly collision = PlanarRegionTools.projectPointToPlanesVertically(new Point3D(closestPointOverall), planarRegionsList);
            collisions.add(new Point3D(collision));
            messager.submitMessage(UIVisibilityGraphsTopics.WalkerCollisionLocations, collisions);

         }

         if (distanceToObstacles < preferredObstacleExtrusionDistance + proximityEpsilon)
            errorMessages += fail("Was too close to an obstacle.");
      }

      if (!errorMessages.isEmpty() && !visualize)
         Assert.assertTrue(errorMessages, false);
      else
         LogTools.info(errorMessages);
   }

   private static String walkerCollisionChecks(Ellipsoid3D walkerShapeWorld, PlanarRegionsList planarRegionsList, List<Point3D> collisionsToPack)
   {
      String errorMessages = "";
      walkerShapeWorld = new Ellipsoid3D(walkerShapeWorld); // Make a copy to ensure we are not modifying the argument

      for (PlanarRegion planarRegion : planarRegionsList.getPlanarRegionsAsList())
      {
         Point3D walkerPosition3D = new Point3D(walkerShapeWorld.getPosition());

         Plane3D plane = planarRegion.getPlane();
         Point3D closestPoint = plane.orthogonalProjectionCopy(walkerPosition3D);

         if (!walkerShapeWorld.isPointInside(closestPoint))
            continue; // Not even close to the region plane, let's keep going.

         Ellipsoid3D walkerShapeLocal = new Ellipsoid3D(walkerShapeWorld);
         planarRegion.transformFromWorldToLocal(walkerShapeLocal);
         walkerPosition3D.set(walkerShapeLocal.getPosition());
         Point2D walkerPosition2D = new Point2D(walkerPosition3D);

         if (planarRegion.getNumberOfConvexPolygons() == 0)
         {
            List<Point2DReadOnly> concaveHullVertices = new ArrayList<>(Arrays.asList(planarRegion.getConcaveHull()));
            double depthThreshold = 0.05;
            List<ConvexPolygon2D> convexPolygons = new ArrayList<>();
            ConcaveHullDecomposition.recursiveApproximateDecomposition(concaveHullVertices, depthThreshold, convexPolygons);
         }

         for (int i = 0; i < planarRegion.getNumberOfConvexPolygons(); i++)
         {
            ConvexPolygon2D convexPolygon = planarRegion.getConvexPolygon(i);
            Point2DBasics closestPoint2D = convexPolygon.orthogonalProjectionCopy(walkerPosition2D);
            if (closestPoint2D == null)
            {
//               if (convexPolygon.isPointInside(walkerPosition2D, -proximityEpsilon))
//               {
//                  errorMessages += fail("Body path is going through a region."); // TODO figure out the proper intersection
                  break;
//               }
//               else
//               {
//                  throw new RuntimeException("Not sure what went wrong to here.");
//               }
            }
            closestPoint = new Point3D(closestPoint2D);

            if (walkerShapeLocal.isPointInside(closestPoint, -proximityEpsilon))
            {
               Point2DBasics intersectionLocal = closestPoint2D;
               Point3D intersectionWorld = new Point3D(intersectionLocal);
               planarRegion.transformFromLocalToWorld(intersectionWorld);
               errorMessages += fail("Body path is going through a region at: " + intersectionWorld);
               collisionsToPack.add(intersectionWorld);
            }
         }
      }
      return errorMessages;
   }

   private static String fail(String message)
   {
      return assertTrue(message, false);
   }

   private static String assertTrue(String message, boolean condition)
   {
      if (visualize || DEBUG)
      {
         if (!condition)
            LogTools.error(message);
      }
      return !condition ? "\n" + message : "";
   }

   private static List<PlanarRegion> createFlatGroundWithWallEnvironment()
   {
      List<PlanarRegion> planarRegions = new ArrayList<>();

      // set up ground plane, 20 x 10
      Point2D groundPlanePointA = new Point2D(10.0, -5.0);
      Point2D groundPlanePointB = new Point2D(10.0, 5.0);
      Point2D groundPlanePointC = new Point2D(-10.0, 5.0);
      Point2D groundPlanePointD = new Point2D(-10.0, -5.0);

      RigidBodyTransform groundTransform = new RigidBodyTransform();
      groundTransform.setTranslation(-10.0, 0.0, 0.0);
      PlanarRegion groundPlaneRegion = new PlanarRegion(groundTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(groundPlanePointA, groundPlanePointB, groundPlanePointC, groundPlanePointD)));

      // set up wall, 5x2
      Point2D wallPointA = new Point2D(2.0, 0.0);
      Point2D wallPointB = new Point2D(0.0, 0.0);
      Point2D wallPointC = new Point2D(2.0, 5.0);
      Point2D wallPointD = new Point2D(0.0, 5.0);

      RigidBodyTransform wallTransform = new RigidBodyTransform();
      wallTransform.setTranslation(-10.0, 0.0, 0.0);
      wallTransform.setRotationPitch(-Math.PI / 2.0);
      PlanarRegion wallRegion = new PlanarRegion(wallTransform,
                                                 new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(wallPointA, wallPointB, wallPointC, wallPointD)));

      planarRegions.add(groundPlaneRegion);
      planarRegions.add(wallRegion);

      return planarRegions;
   }

   private static List<PlanarRegion> createFlatGroundWithWallOpeningEnvironment()
   {
      List<PlanarRegion> planarRegions = new ArrayList<>();

      // set up ground plane, 20 x 10
      Point2D groundPlanePointA = new Point2D(10.0, -5.0);
      Point2D groundPlanePointB = new Point2D(10.0, 5.0);
      Point2D groundPlanePointC = new Point2D(-10.0, 5.0);
      Point2D groundPlanePointD = new Point2D(-10.0, -5.0);

      RigidBodyTransform groundTransform = new RigidBodyTransform();
      groundTransform.setTranslation(-10.0, 0.0, 0.0);
      PlanarRegion groundPlaneRegion = new PlanarRegion(groundTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(groundPlanePointA, groundPlanePointB, groundPlanePointC, groundPlanePointD)));

      // set up wall, 5x2
      Point2D wallPointA = new Point2D(2.0, 0.0);
      Point2D wallPointB = new Point2D(0.0, 0.0);
      Point2D wallPointC = new Point2D(2.0, 4.5);
      Point2D wallPointD = new Point2D(0.0, 4.5);

      RigidBodyTransform leftWallTransform = new RigidBodyTransform();
      leftWallTransform.setTranslation(-10.0, 0.5, 0.0);
      leftWallTransform.setRotationPitch(-Math.PI / 2.0);
      PlanarRegion leftWallRegion = new PlanarRegion(leftWallTransform,
                                                     new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(wallPointA, wallPointB, wallPointC, wallPointD)));

      RigidBodyTransform rightWallTransform = new RigidBodyTransform();
      rightWallTransform.setTranslation(-10.0, -5.0, 0.0);
      rightWallTransform.setRotationPitch(-Math.PI / 2.0);
      PlanarRegion rightWallRegion = new PlanarRegion(rightWallTransform,
                                                      new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(wallPointA, wallPointB, wallPointC, wallPointD)));

      planarRegions.add(groundPlaneRegion);
      planarRegions.add(leftWallRegion);
      planarRegions.add(rightWallRegion);

      return planarRegions;
   }

   private static List<PlanarRegion> createFlatGroundWithWallAwkwardOpeningEnvironment()
   {
      List<PlanarRegion> planarRegions = new ArrayList<>();

      // set up ground plane, 20 x 10
      Point2D groundPlanePointA = new Point2D(10.0, -5.0);
      Point2D groundPlanePointB = new Point2D(10.0, 5.0);
      Point2D groundPlanePointC = new Point2D(-10.0, 5.0);
      Point2D groundPlanePointD = new Point2D(-10.0, -5.0);

      RigidBodyTransform groundTransform = new RigidBodyTransform();
      groundTransform.setTranslation(-10.0, 0.0, 0.0);
      PlanarRegion groundPlaneRegion = new PlanarRegion(groundTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(groundPlanePointA, groundPlanePointB, groundPlanePointC, groundPlanePointD)));

      // set up wall, 5x2
      Point2D wallPointA = new Point2D(2.0, 0.0);
      Point2D wallPointB = new Point2D(0.0, 0.0);
      Point2D wallPointC = new Point2D(2.0, 4.25);
      Point2D wallPointD = new Point2D(0.0, 4.25);

      RigidBodyTransform leftWallTransform = new RigidBodyTransform();
      leftWallTransform.setTranslation(-10.0, 0.75, 0.0);
      leftWallTransform.setRotationPitch(-Math.PI / 2.0);
      PlanarRegion leftWallRegion = new PlanarRegion(leftWallTransform,
                                                     new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(wallPointA, wallPointB, wallPointC, wallPointD)));

      RigidBodyTransform rightWallTransform = new RigidBodyTransform();
      rightWallTransform.setTranslation(-10.0, -5.0, 0.0);
      rightWallTransform.setRotationPitch(-Math.PI / 2.0);
      PlanarRegion rightWallRegion = new PlanarRegion(rightWallTransform,
                                                      new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(wallPointA, wallPointB, wallPointC, wallPointD)));

      planarRegions.add(groundPlaneRegion);
      planarRegions.add(leftWallRegion);
      planarRegions.add(rightWallRegion);

      return planarRegions;
   }

   private static List<PlanarRegion> createFlatGroundWithBoxEnvironment()
   {
      List<PlanarRegion> planarRegions = new ArrayList<>();

      // set up ground plane, 20 x 10
      Point2D groundPlanePointA = new Point2D(10.0, -5.0);
      Point2D groundPlanePointB = new Point2D(10.0, 5.0);
      Point2D groundPlanePointC = new Point2D(-10.0, 5.0);
      Point2D groundPlanePointD = new Point2D(-10.0, -5.0);

      RigidBodyTransform groundTransform = new RigidBodyTransform();
      groundTransform.setTranslation(-10.0, 0.0, 0.0);
      PlanarRegion groundPlaneRegion = new PlanarRegion(groundTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(groundPlanePointA, groundPlanePointB, groundPlanePointC, groundPlanePointD)));

      // set up wall, 5x2
      Point2D frontWallPointA = new Point2D(2.0, 0.0);
      Point2D frontWallPointB = new Point2D(0.0, 0.0);
      Point2D frontWallPointC = new Point2D(2.0, 5.0);
      Point2D frontWallPointD = new Point2D(0.0, 5.0);

      RigidBodyTransform frontWallTransform = new RigidBodyTransform();
      frontWallTransform.setTranslation(-11.5, 0.0, 0.0);
      frontWallTransform.setRotationPitch(-Math.PI / 2.0);
      PlanarRegion frontWallRegion = new PlanarRegion(frontWallTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(frontWallPointA, frontWallPointB, frontWallPointC, frontWallPointD)));

      RigidBodyTransform backWallTransform = new RigidBodyTransform();
      backWallTransform.setTranslation(-8.5, 0.0, 0.0);
      backWallTransform.setRotationPitch(-Math.PI / 2.0);
      PlanarRegion backWallRegion = new PlanarRegion(backWallTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(frontWallPointA, frontWallPointB, frontWallPointC, frontWallPointD)));

      Point2D sideWallPointA = new Point2D(2.0, 0.0);
      Point2D sideWallPointB = new Point2D(0.0, 0.0);
      Point2D sideWallPointC = new Point2D(2.0, 3.0);
      Point2D sideWallPointD = new Point2D(0.0, 3.0);

      RigidBodyTransform sideWallTransform = new RigidBodyTransform();
      sideWallTransform.setTranslation(-11.5, 0.0, 0.0);
      sideWallTransform.setRotationYawPitchRoll(-Math.PI / 2.0, -Math.PI / 2.0, 0.0);
      PlanarRegion sideWallRegion = new PlanarRegion(sideWallTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(sideWallPointA, sideWallPointB, sideWallPointC, sideWallPointD)));

      planarRegions.add(groundPlaneRegion);
      planarRegions.add(frontWallRegion);
      planarRegions.add(backWallRegion);
      planarRegions.add(sideWallRegion);

      return planarRegions;
   }

   private static List<PlanarRegion> createFlatGroundWithBoxInMiddleEnvironment()
   {
      List<PlanarRegion> planarRegions = new ArrayList<>();

      // set up ground plane, 20 x 10
      Point2D groundPlanePointA = new Point2D(10.0, -5.0);
      Point2D groundPlanePointB = new Point2D(10.0, 5.0);
      Point2D groundPlanePointC = new Point2D(-10.0, 5.0);
      Point2D groundPlanePointD = new Point2D(-10.0, -5.0);

      RigidBodyTransform groundTransform = new RigidBodyTransform();
      groundTransform.setTranslation(-10.0, 0.0, 0.0);
      PlanarRegion groundPlaneRegion = new PlanarRegion(groundTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(groundPlanePointA, groundPlanePointB, groundPlanePointC, groundPlanePointD)));

      // set up wall, 5x2
      Point2D frontWallPointA = new Point2D(2.0, 0.0);
      Point2D frontWallPointB = new Point2D(0.0, 0.0);
      Point2D frontWallPointC = new Point2D(2.0, 9.0);
      Point2D frontWallPointD = new Point2D(0.0, 9.0);

      RigidBodyTransform frontWallTransform = new RigidBodyTransform();
      frontWallTransform.setTranslation(-11.5, -4.5, 0.0);
      frontWallTransform.setRotationPitch(-Math.PI / 2.0);
      PlanarRegion frontWallRegion = new PlanarRegion(frontWallTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(frontWallPointA, frontWallPointB, frontWallPointC, frontWallPointD)));

      RigidBodyTransform backWallTransform = new RigidBodyTransform();
      backWallTransform.setTranslation(-8.5, -4.5, 0.0);
      backWallTransform.setRotationPitch(-Math.PI / 2.0);
      PlanarRegion backWallRegion = new PlanarRegion(backWallTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(frontWallPointA, frontWallPointB, frontWallPointC, frontWallPointD)));

      Point2D sideWallPointA = new Point2D(2.0, 0.0);
      Point2D sideWallPointB = new Point2D(0.0, 0.0);
      Point2D sideWallPointC = new Point2D(2.0, 3.0);
      Point2D sideWallPointD = new Point2D(0.0, 3.0);

      RigidBodyTransform leftSideWall = new RigidBodyTransform();
      leftSideWall.setTranslation(-11.5, 4.5, 0.0);
      leftSideWall.setRotationYawPitchRoll(-Math.PI / 2.0, -Math.PI / 2.0, 0.0);
      PlanarRegion leftSideWallRegion = new PlanarRegion(leftSideWall, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(sideWallPointA, sideWallPointB, sideWallPointC, sideWallPointD)));

      RigidBodyTransform rightSideWall = new RigidBodyTransform();
      rightSideWall.setTranslation(-11.5, -4.5, 0.0);
      rightSideWall.setRotationYawPitchRoll(-Math.PI / 2.0, -Math.PI / 2.0, 0.0);
      PlanarRegion rightSideWallRegion = new PlanarRegion(rightSideWall, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(sideWallPointA, sideWallPointB, sideWallPointC, sideWallPointD)));

      planarRegions.add(groundPlaneRegion);
      planarRegions.add(frontWallRegion);
      planarRegions.add(backWallRegion);
      planarRegions.add(leftSideWallRegion);
      planarRegions.add(rightSideWallRegion);

      return planarRegions;
   }

   private static List<PlanarRegion> createFlatGroundWithBoxInMiddleEnvironmentButIslandToTheLeft()
   {
      List<PlanarRegion> planarRegions = new ArrayList<>();

      // set up ground plane, 20 x 10
      Point2D groundPlanePointA = new Point2D(10.0, -5.0);
      Point2D groundPlanePointB = new Point2D(10.0, 5.0);
      Point2D groundPlanePointC = new Point2D(-10.0, 5.0);
      Point2D groundPlanePointD = new Point2D(-10.0, -5.0);

      RigidBodyTransform groundTransform = new RigidBodyTransform();
      groundTransform.setTranslation(-10.0, 0.0, 0.0);
      PlanarRegion groundPlaneRegion = new PlanarRegion(groundTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(groundPlanePointA, groundPlanePointB, groundPlanePointC, groundPlanePointD)));

      // set up island plane, 20 x 10
      RigidBodyTransform islandTransform = new RigidBodyTransform();
      islandTransform.setTranslation(-10.0, 10.2, 0.0);
      PlanarRegion islandPlaneRegion = new PlanarRegion(islandTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(groundPlanePointA, groundPlanePointB, groundPlanePointC, groundPlanePointD)));

      // set up wall, 5x2
      Point2D frontWallPointA = new Point2D(2.0, 0.0);
      Point2D frontWallPointB = new Point2D(0.0, 0.0);
      Point2D frontWallPointC = new Point2D(2.0, 9.0);
      Point2D frontWallPointD = new Point2D(0.0, 9.0);

      RigidBodyTransform frontWallTransform = new RigidBodyTransform();
      frontWallTransform.setTranslation(-11.5, -4.5, 0.0);
      frontWallTransform.setRotationPitch(-Math.PI / 2.0);
      PlanarRegion frontWallRegion = new PlanarRegion(frontWallTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(frontWallPointA, frontWallPointB, frontWallPointC, frontWallPointD)));

      RigidBodyTransform backWallTransform = new RigidBodyTransform();
      backWallTransform.setTranslation(-8.5, -4.5, 0.0);
      backWallTransform.setRotationPitch(-Math.PI / 2.0);
      PlanarRegion backWallRegion = new PlanarRegion(backWallTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(frontWallPointA, frontWallPointB, frontWallPointC, frontWallPointD)));

      Point2D sideWallPointA = new Point2D(2.0, 0.0);
      Point2D sideWallPointB = new Point2D(0.0, 0.0);
      Point2D sideWallPointC = new Point2D(2.0, 3.0);
      Point2D sideWallPointD = new Point2D(0.0, 3.0);

      RigidBodyTransform leftSideWall = new RigidBodyTransform();
      leftSideWall.setTranslation(-11.5, 4.5, 0.0);
      leftSideWall.setRotationYawPitchRoll(-Math.PI / 2.0, -Math.PI / 2.0, 0.0);
      PlanarRegion leftSideWallRegion = new PlanarRegion(leftSideWall, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(sideWallPointA, sideWallPointB, sideWallPointC, sideWallPointD)));

      RigidBodyTransform rightSideWall = new RigidBodyTransform();
      rightSideWall.setTranslation(-11.5, -4.5, 0.0);
      rightSideWall.setRotationYawPitchRoll(-Math.PI / 2.0, -Math.PI / 2.0, 0.0);
      PlanarRegion rightSideWallRegion = new PlanarRegion(rightSideWall, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(sideWallPointA, sideWallPointB, sideWallPointC, sideWallPointD)));

      planarRegions.add(groundPlaneRegion);
      planarRegions.add(islandPlaneRegion);
      planarRegions.add(frontWallRegion);
      planarRegions.add(backWallRegion);
      planarRegions.add(leftSideWallRegion);
      planarRegions.add(rightSideWallRegion);

      return planarRegions;
   }

   private static List<PlanarRegion> createFlatGroundWithBoxesEnvironment()
   {
      List<PlanarRegion> planarRegions = new ArrayList<>();

      // set up ground plane, 20 x 10
      Point2D groundPlanePointA = new Point2D(10.0, -5.0);
      Point2D groundPlanePointB = new Point2D(10.0, 5.0);
      Point2D groundPlanePointC = new Point2D(-10.0, 5.0);
      Point2D groundPlanePointD = new Point2D(-10.0, -5.0);

      RigidBodyTransform groundTransform = new RigidBodyTransform();
      groundTransform.setTranslation(-10.0, 0.0, 0.0);
      PlanarRegion groundPlaneRegion = new PlanarRegion(groundTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(groundPlanePointA, groundPlanePointB, groundPlanePointC, groundPlanePointD)));

      // set up wall, 5x2
      Point2D frontWallPointA = new Point2D(2.0, 0.0);
      Point2D frontWallPointB = new Point2D(0.0, 0.0);
      Point2D frontWallPointC = new Point2D(2.0, 4.5);
      Point2D frontWallPointD = new Point2D(0.0, 4.5);

      RigidBodyTransform frontLeftWallTransform = new RigidBodyTransform();
      frontLeftWallTransform.setTranslation(-11.5, 0.5, 0.0);
      frontLeftWallTransform.setRotationPitch(-Math.PI / 2.0);
      PlanarRegion frontLeftWallRegion = new PlanarRegion(frontLeftWallTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(frontWallPointA, frontWallPointB, frontWallPointC, frontWallPointD)));

      RigidBodyTransform frontRightWallTransform = new RigidBodyTransform();
      frontRightWallTransform.setTranslation(-11.5, -5.0, 0.0);
      frontRightWallTransform.setRotationPitch(-Math.PI / 2.0);
      PlanarRegion frontRightWallRegion = new PlanarRegion(frontRightWallTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(frontWallPointA, frontWallPointB, frontWallPointC, frontWallPointD)));

      // set up wall, 5x2
      Point2D backWallPointA = new Point2D(2.0, 0.0);
      Point2D backWallPointB = new Point2D(0.0, 0.0);
      Point2D backWallPointC = new Point2D(2.0, 4.25);
      Point2D backWallPointD = new Point2D(0.0, 4.25);

      RigidBodyTransform backLeftWallTransform = new RigidBodyTransform();
      backLeftWallTransform.setTranslation(-8.5, 0.75, 0.0);
      backLeftWallTransform.setRotationPitch(-Math.PI / 2.0);
      PlanarRegion backLeftWallRegion = new PlanarRegion(backLeftWallTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(backWallPointA, backWallPointB, backWallPointC, backWallPointD)));

      RigidBodyTransform backRightWallTransform = new RigidBodyTransform();
      backRightWallTransform.setTranslation(-8.5, -5.0, 0.0);
      backRightWallTransform.setRotationPitch(-Math.PI / 2.0);
      PlanarRegion backRightWallRegion = new PlanarRegion(backRightWallTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(backWallPointA, backWallPointB, backWallPointC, backWallPointD)));

      Point2D sideWallPointA = new Point2D(2.0, 0.0);
      Point2D sideWallPointB = new Point2D(0.0, 0.0);
      Point2D sideWallPointC = new Point2D(2.0, 3.0);
      Point2D sideWallPointD = new Point2D(0.0, 3.0);

      RigidBodyTransform leftSideWallTransform = new RigidBodyTransform();
      leftSideWallTransform.setTranslation(-11.5, 0.625, 0.0);
      leftSideWallTransform.setRotationYawPitchRoll(-Math.PI / 2.0, -Math.PI / 2.0, 0.0);
      PlanarRegion leftSideWallRegion = new PlanarRegion(leftSideWallTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(sideWallPointA, sideWallPointB, sideWallPointC, sideWallPointD)));

      planarRegions.add(groundPlaneRegion);
      planarRegions.add(frontLeftWallRegion);
      planarRegions.add(backLeftWallRegion);
      planarRegions.add(frontRightWallRegion);
      planarRegions.add(backRightWallRegion);
      //      planarRegions.add(leftSideWallRegion);

      return planarRegions;
   }

   private static List<PlanarRegion> createFlatGroundTwoDifferentWidthWallsEnvironment()
   {
      List<PlanarRegion> planarRegions = new ArrayList<>();

      // set up ground plane, 20 x 10
      Point2D groundPlanePointA = new Point2D(10.0, -5.0);
      Point2D groundPlanePointB = new Point2D(10.0, 5.0);
      Point2D groundPlanePointC = new Point2D(-10.0, 5.0);
      Point2D groundPlanePointD = new Point2D(-10.0, -5.0);

      RigidBodyTransform groundTransform = new RigidBodyTransform();
      groundTransform.setTranslation(-10.0, 0.0, 0.0);
      PlanarRegion groundPlaneRegion = new PlanarRegion(groundTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(groundPlanePointA, groundPlanePointB, groundPlanePointC, groundPlanePointD)));

      // set up wall, 5x2
      Point2D wallPointA = new Point2D(2.0, 0.0);
      Point2D wallPointB = new Point2D(0.0, 0.0);
      Point2D wallPointC = new Point2D(2.0, 5.0);
      Point2D wallPointD = new Point2D(0.0, 5.0);

      RigidBodyTransform wallTransform = new RigidBodyTransform();
      wallTransform.setTranslation(-8.5, 0.0, 0.0);
      wallTransform.setRotationPitch(-Math.PI / 2.0);
      PlanarRegion wallRegion = new PlanarRegion(wallTransform,
                                                 new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(wallPointA, wallPointB, wallPointC, wallPointD)));

      Point2D otherWallPointA = new Point2D(2.0, 0.0);
      Point2D otherWallPointB = new Point2D(0.0, 0.0);
      Point2D otherWallPointC = new Point2D(2.0, 4.5);
      Point2D otherWallPointD = new Point2D(0.0, 4.5);

      RigidBodyTransform otherWallTransform = new RigidBodyTransform();
      otherWallTransform.setTranslation(-11.5, 0.5, 0.0);
      otherWallTransform.setRotationPitch(-Math.PI / 2.0);
      PlanarRegion otherWallRegion = new PlanarRegion(otherWallTransform, new ConvexPolygon2D(
            Vertex2DSupplier.asVertex2DSupplier(otherWallPointA, otherWallPointB, otherWallPointC, otherWallPointD)));

      planarRegions.add(groundPlaneRegion);
      planarRegions.add(wallRegion);
      planarRegions.add(otherWallRegion);

      return planarRegions;
   }


   private static void visualize(List<? extends Pose3DReadOnly> path, VisibilityGraphsParametersReadOnly parameters, PlanarRegionsList planarRegionsList, Point3D start, Point3D goal, List<VisibilityMapWithNavigableRegion> navigableRegions)
   {
      Random random = new Random(324);
      planarRegionsList.getPlanarRegionsAsList().forEach(region -> region.setRegionId(random.nextInt()));

      List<Point3DReadOnly> pathPoints = path.stream().map(pose -> new Point3D(pose.getPosition())).collect(Collectors.toList());
      visualizerApplication.submitPlanarRegionsListToVisualizer(planarRegionsList);
      visualizerApplication.submitGoalToVisualizer(goal);
      visualizerApplication.submitStartToVisualizer(start);
      visualizerApplication.submitNavigableRegionsToVisualizer(navigableRegions);
      messager.submitMessage(UIVisibilityGraphsTopics.BodyPathData, pathPoints);

      while (true)
      {
         checkPath(path, start, goal, parameters, planarRegionsList, navigableRegions);
      }
   }

   private VisibilityGraphsParametersReadOnly createVisibilityGraphParametersForTest()
   {
      VisibilityGraphsParametersBasics parameters = new DefaultVisibilityGraphParameters()
      {
         @Override
         public PlanarRegionFilter getPlanarRegionFilter()
         {
            return new PlanarRegionFilter()
            {
               @Override
               public boolean isPlanarRegionRelevant(PlanarRegion region)
               {
                  return true;
               }
            };
         }



         @Override
         public NavigableExtrusionDistanceCalculator getNavigableExtrusionDistanceCalculator()
            {
               return new NavigableExtrusionDistanceCalculator()
               {
                  @Override
                  public double computeNavigableExtrusionDistance(PlanarRegion navigableRegionToBeExtruded)
                  {
                     return 0.01;
                  }
               };
            }
         };
      parameters.setObstacleExtrusionDistance(obstacleExtrusionDistance);
      parameters.setPreferredObstacleExtrusionDistance(preferredObstacleExtrusionDistance);
      parameters.setClusterResolution(0.501);
      parameters.setPerformPostProcessingNodeShifting(true);
      parameters.setComputeOrientationsToAvoidObstacles(true);

      return parameters;
   }

}
