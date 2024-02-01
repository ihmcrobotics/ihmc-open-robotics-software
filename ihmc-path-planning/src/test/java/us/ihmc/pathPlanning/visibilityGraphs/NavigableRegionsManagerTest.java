package us.ihmc.pathPlanning.visibilityGraphs;

import static us.ihmc.robotics.Assert.assertFalse;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.shape.primitives.Ellipsoid3D;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanHolder;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.ExtrusionHull;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityGraphNavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMapSolution;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMapWithNavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.graphSearch.EstimatedCostToGoal;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.NavigableExtrusionDistanceCalculator;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.BodyPathPostProcessor;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.ObstacleAvoidanceProcessor;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.PathOrientationCalculator;
import us.ihmc.pathPlanning.visibilityGraphs.tools.TestEnvironmentTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityTools;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullDecomposition;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionFilter;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;

public class NavigableRegionsManagerTest
{
   private static boolean visualize = false;
   private static final double epsilon = 1e-4;
   private static final double proximityEpsilon = 6e-2;

   // For enabling helpful prints.
   private static boolean DEBUG = false;

   private static VisibilityGraphsTestVisualizerApplication visualizerApplication = null;
   // Because we use JavaFX, there will be two instance of VisibilityGraphsFrameworkTest, one for running the test and one starting the ui. The messager has to be static so both the ui and test use the same instance.
   private static JavaFXMessager messager = null;

   // The following are used for collision checks.
   private static final double walkerOffsetHeight = 0.75;

//   private static final Vector3D walkerRadii = new Vector3D(0.25, 0.25, 0.5);

   private static final double obstacleExtrusionDistance = 0.2;
   private static final double preferredObstacleExtrusionDistance = 1.0;
   private static final Vector3D walkerRadii = new Vector3D(obstacleExtrusionDistance, obstacleExtrusionDistance, 0.5);
   private static final Vector3D walkerBox = new Vector3D(2.0 * obstacleExtrusionDistance, 2.0 * obstacleExtrusionDistance, 1.0);

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

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(TestEnvironmentTools.createFlatGroundWithWallEnvironment());

      // test aligned with the edge of the wall, requiring slight offset
      Point3D start = new Point3D(-15.0, 0.0, 0.0);
      Point3D goal = new Point3D(-5.0, 0.0, 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      BodyPathPostProcessor postProcessor = new ObstacleAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator
            .computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution(), new Quaternion(), new Quaternion());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList(), navigableRegionsManager.getVisibilityMapSolution());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testFlatGroundWithWallOnOppositeSidesOfWall()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(TestEnvironmentTools.createFlatGroundWithWallEnvironment());

      // test on opposite sides of the wall, requiring going around it
      Point3D start = new Point3D(-15.0, 1.0, 0.0);
      Point3D goal = new Point3D(-5.0, 1.0, 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      BodyPathPostProcessor postProcessor = new ObstacleAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator
            .computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution(), new Quaternion(), new Quaternion());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList(), navigableRegionsManager.getVisibilityMapSolution());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testFlatGroundWithWallStraightShotButVeryNearWall()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(TestEnvironmentTools.createFlatGroundWithWallEnvironment());

      // test on opposite sides of the wall, requiring going around it
      Point3D start = new Point3D(-15.0, -0.05 * parameters.getObstacleExtrusionDistance(), 0.0);
      Point3D goal = new Point3D(-5.0, -0.05 * parameters.getObstacleExtrusionDistance(), 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      BodyPathPostProcessor postProcessor = new ObstacleAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator
            .computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution(), new Quaternion(), new Quaternion());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList(), navigableRegionsManager.getVisibilityMapSolution());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testFlatGroundWithWallStraightShotButNearWall()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(TestEnvironmentTools.createFlatGroundWithWallEnvironment());

      // test on opposite sides of the wall, requiring going around it
      Point3D start = new Point3D(-15.0, -0.1 * parameters.getObstacleExtrusionDistance(), 0.0);
      Point3D goal = new Point3D(-5.0, -0.1 * parameters.getObstacleExtrusionDistance(), 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      BodyPathPostProcessor postProcessor = new ObstacleAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator
            .computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution(), new Quaternion(), new Quaternion());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList(), navigableRegionsManager.getVisibilityMapSolution());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testFlatGroundWithWallAlmostStraightShot()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(TestEnvironmentTools.createFlatGroundWithWallEnvironment());

      // test on opposite sides of the wall, requiring going around it
      Point3D start = new Point3D(-15.0, -0.95 * parameters.getObstacleExtrusionDistance(), 0.0);
      Point3D goal = new Point3D(-5.0, -0.95 * parameters.getObstacleExtrusionDistance(), 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      BodyPathPostProcessor postProcessor = new ObstacleAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator
            .computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution(), new Quaternion(), new Quaternion());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList(), navigableRegionsManager.getVisibilityMapSolution());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testFlatGroundWithWallStraightShot()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(TestEnvironmentTools.createFlatGroundWithWallEnvironment());

      // test on opposite sides of the wall, requiring going around it
      Point3D start = new Point3D(-15.0, -1.05 * parameters.getObstacleExtrusionDistance(), 0.0);
      Point3D goal = new Point3D(-5.0, -1.05 * parameters.getObstacleExtrusionDistance(), 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      BodyPathPostProcessor postProcessor = new ObstacleAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator
            .computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution(), new Quaternion(), new Quaternion());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList(), navigableRegionsManager.getVisibilityMapSolution());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testFlatGroundWithBoxInlineWithWall()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(TestEnvironmentTools.createFlatGroundWithBoxEnvironment());

      // test aligned with the edge of the wall, requiring slight offset
      Point3D start = new Point3D(-15.0, 0.0, 0.0);
      Point3D goal = new Point3D(-5.0, 0.0, 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      BodyPathPostProcessor postProcessor = new ObstacleAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator
            .computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution(), new Quaternion(), new Quaternion());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList(), navigableRegionsManager.getVisibilityMapSolution());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testFlatGroundWithBoxOnOppositeSidesOfWall()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(TestEnvironmentTools.createFlatGroundWithBoxEnvironment());

      // test on opposite sides of the wall, requiring going around it
      Point3D start = new Point3D(-15.0, 1.0, 0.0);
      Point3D goal = new Point3D(-5.0, 1.0, 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      BodyPathPostProcessor postProcessor = new ObstacleAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator
            .computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution(), new Quaternion(), new Quaternion());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList(), navigableRegionsManager.getVisibilityMapSolution());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testFlatGroundWithBoxStraightShotButVeryNearWall()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(TestEnvironmentTools.createFlatGroundWithBoxEnvironment());

      // test on opposite sides of the wall, requiring going around it
      Point3D start = new Point3D(-15.0, -0.05 * parameters.getObstacleExtrusionDistance(), 0.0);
      Point3D goal = new Point3D(-5.0, -0.05 * parameters.getObstacleExtrusionDistance(), 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      BodyPathPostProcessor postProcessor = new ObstacleAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator
            .computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution(), new Quaternion(), new Quaternion());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList(), navigableRegionsManager.getVisibilityMapSolution());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testFlatGroundWithBoxStraightShotButNearWall()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(TestEnvironmentTools.createFlatGroundWithBoxEnvironment());

      // test on opposite sides of the wall, requiring going around it
      Point3D start = new Point3D(-15.0, -0.1 * parameters.getObstacleExtrusionDistance(), 0.0);
      Point3D goal = new Point3D(-5.0, -0.1 * parameters.getObstacleExtrusionDistance(), 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      BodyPathPostProcessor postProcessor = new ObstacleAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator
            .computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution(), new Quaternion(), new Quaternion());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList(), navigableRegionsManager.getVisibilityMapSolution());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testFlatGroundWithBoxAlmostStraightShot()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(TestEnvironmentTools.createFlatGroundWithBoxEnvironment());

      // test on opposite sides of the wall, requiring going around it
      Point3D start = new Point3D(-15.0, -0.95 * parameters.getObstacleExtrusionDistance(), 0.0);
      Point3D goal = new Point3D(-5.0, -0.95 * parameters.getObstacleExtrusionDistance(), 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      BodyPathPostProcessor postProcessor = new ObstacleAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator
            .computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution(), new Quaternion(), new Quaternion());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList(), navigableRegionsManager.getVisibilityMapSolution());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testFlatGroundWithBoxStraightShot()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(TestEnvironmentTools.createFlatGroundWithBoxEnvironment());

      // test on opposite sides of the wall, requiring going around it
      Point3D start = new Point3D(-15.0, -1.05 * parameters.getObstacleExtrusionDistance(), 0.0);
      Point3D goal = new Point3D(-5.0, -1.05 * parameters.getObstacleExtrusionDistance(), 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      BodyPathPostProcessor postProcessor = new ObstacleAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator
            .computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution(), new Quaternion(), new Quaternion());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList(), navigableRegionsManager.getVisibilityMapSolution());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testFlatGroundWithTwoDifferentWalls()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(TestEnvironmentTools.createFlatGroundTwoDifferentWidthWallsEnvironment());

      // test straight shot, initially going to one of the nodes
      Point3D start = new Point3D(-15.0, -0.5 * parameters.getObstacleExtrusionDistance(), 0.0);
      Point3D goal = new Point3D(-5.0, -0.5 * parameters.getObstacleExtrusionDistance(), 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      BodyPathPostProcessor postProcessor = new ObstacleAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator
            .computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution(), new Quaternion(), new Quaternion());

      /*
      if (visualize)
      {
         visualize(path, planarRegionsList, start, goal);
      }
      */

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());

      start = new Point3D(-15.0, 1.0 * parameters.getObstacleExtrusionDistance(), 0.0);

      path = navigableRegionsManager.calculateBodyPath(start, goal);
      posePath = orientationCalculator.computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution(), new Quaternion(), new Quaternion());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList(), navigableRegionsManager.getVisibilityMapSolution());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testFlatGroundBetweenWallOpening()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(TestEnvironmentTools.createFlatGroundWithWallOpeningEnvironment());

      // test aligned with the edge of the wall, requiring slight offset
      Point3D start = new Point3D(-15.0, 1.5, 0.0);
      Point3D goal = new Point3D(-5.0, 1.5, 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      BodyPathPostProcessor postProcessor = new ObstacleAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator
            .computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution(), new Quaternion(), new Quaternion());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList(), navigableRegionsManager.getVisibilityMapSolution());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testFlatGroundBetweenWallOpeningStraightShot()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(TestEnvironmentTools.createFlatGroundWithWallOpeningEnvironment());

      // test aligned with the edge of the wall, requiring slight offset
      Point3D start = new Point3D(-15.0, 0.0, 0.0);
      Point3D goal = new Point3D(-5.0, 0.0, 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      BodyPathPostProcessor postProcessor = new ObstacleAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator
            .computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution(), new Quaternion(), new Quaternion());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList(), navigableRegionsManager.getVisibilityMapSolution());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testFlatGroundBetweenAwkwardWallOpening()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(TestEnvironmentTools.createFlatGroundWithWallAwkwardOpeningEnvironment());

      // test aligned with the edge of the wall, requiring slight offset
      Point3D start = new Point3D(-15.0, 1.5, 0.0);
      Point3D goal = new Point3D(-5.0, 1.5, 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      BodyPathPostProcessor postProcessor = new ObstacleAvoidanceProcessor(parameters);

      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator
            .computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution(), new Quaternion(), new Quaternion());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList(), navigableRegionsManager.getVisibilityMapSolution());
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

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(TestEnvironmentTools.createFlatGroundWithBoxesEnvironment());

      // test aligned with the edge of the wall, requiring slight offset
      Point3D start = new Point3D(-15.0, 1.5, 0.0);
      Point3D goal = new Point3D(-5.0, 1.5, 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      BodyPathPostProcessor postProcessor = new ObstacleAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator
            .computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution(), new Quaternion(), new Quaternion());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList(), navigableRegionsManager.getVisibilityMapSolution());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testFlatGroundBetweenBoxesOpeningStraightShot()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(TestEnvironmentTools.createFlatGroundWithBoxesEnvironment());

      // test aligned with the edge of the wall, requiring slight offset
      Point3D start = new Point3D(-15.0, 0.0, 0.0);
      Point3D goal = new Point3D(-5.0, 0.0, 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      BodyPathPostProcessor postProcessor = new ObstacleAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator
            .computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution(), new Quaternion(), new Quaternion());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList(), navigableRegionsManager.getVisibilityMapSolution());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testFlatGroundBetweenBoxInMiddle()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(TestEnvironmentTools.createFlatGroundWithBoxInMiddleEnvironment());

      // test aligned with the edge of the wall, requiring slight offset
      Point3D start = new Point3D(-15.0, 0.0, 0.0);
      Point3D goal = new Point3D(-5.0, 0.0, 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      BodyPathPostProcessor postProcessor = new ObstacleAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator
            .computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution(), new Quaternion(), new Quaternion());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList(), navigableRegionsManager.getVisibilityMapSolution());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList(), 0.15);
   }

   @Test
   public void testFlatGroundBetweenBoxInMiddleButExtraSupportToTheLeft()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(TestEnvironmentTools.createFlatGroundWithBoxInMiddleEnvironmentButIslandToTheLeft());

      // test aligned with the edge of the wall, requiring slight offset
      Point3D start = new Point3D(-15.0, 0.0, 0.0);
      Point3D goal = new Point3D(-5.0, 0.0, 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      BodyPathPostProcessor postProcessor = new ObstacleAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator
            .computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution(), new Quaternion(), new Quaternion());

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList(), navigableRegionsManager.getVisibilityMapSolution());
      }
   }

   @Test
   public void testStairs()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();
      ((VisibilityGraphsParametersBasics) parameters).setPerformPostProcessingNodeShifting(false);
      ((VisibilityGraphsParametersBasics) parameters).setMaxInterRegionConnectionLength(0.25);

      double heightDelta = 0.1;
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.addRectangle(0.3, 1.0);
      generator.translate(0.3, 0.0, heightDelta);
      generator.addRectangle(0.3, 1.0);
      generator.translate(0.3, 0.0, heightDelta);
      generator.addRectangle(0.3, 1.0);
      generator.translate(0.65, 0.0, heightDelta);
      generator.addRectangle(1.0, 1.0);
      generator.translate(0.65, 0.0, -heightDelta);
      generator.addRectangle(0.3, 1.0);
      generator.translate(0.3, 0.0, -heightDelta);
      generator.addRectangle(0.3, 1.0);
      generator.translate(0.3, 0.0, -heightDelta);
      generator.addRectangle(0.3, 1.0);

      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      // test aligned with the edge of the wall, requiring slight offset
      Point3D start = new Point3D(0.0, 0.0, 0.0);
      Point3D goal = new Point3D(2.5, 0.0, 0.0);

      Point3D start2 = new Point3D(0.3, 0.0, 0.1);
      Point3D start3 = new Point3D(0.6, 0.0, 0.2);
      Point3D start4 = new Point3D(0.95, 0.0, 0.3);

      EstimatedCostToGoal heuristic = new EstimatedCostToGoal(parameters);
      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      BodyPathPostProcessor postProcessor = new ObstacleAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor, heuristic);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator
            .computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution(), new Quaternion(), new Quaternion());

      VisibilityGraphNavigableRegion region1 = navigableRegionsManager.getVisibilityGraph().getVisibilityGraphNavigableRegionContainingThisPoint(start, 2.0, 0.1);
      VisibilityGraphNavigableRegion region2 = navigableRegionsManager.getVisibilityGraph().getVisibilityGraphNavigableRegionContainingThisPoint(start2, 2.0, 0.1);
      VisibilityGraphNavigableRegion region3 = navigableRegionsManager.getVisibilityGraph().getVisibilityGraphNavigableRegionContainingThisPoint(start3, 2.0, 0.1);

      List<Point3DReadOnly> expectedPath = new ArrayList<>();
      expectedPath.add(getClosestPointOnClusterToPoint(region1.getNavigableRegion().getObstacleClusters().get(0).getNavigableExtrusionsInWorld(), start));
      expectedPath.add(getClosestPointOnClusterToPoint(region2.getNavigableRegion().getHomeRegionCluster().getNavigableExtrusionsInWorld(), expectedPath.get(0)));
      expectedPath.add(getClosestPointOnClusterToPoint(region2.getNavigableRegion().getObstacleClusters().get(0).getNavigableExtrusionsInWorld(), expectedPath.get(1)));

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList(), navigableRegionsManager.getVisibilityMapSolution(), 1.0);
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList(), 1.0);
   }


   public Point3DReadOnly getClosestPointOnClusterToPoint(List<Point3DReadOnly> points, Point3DReadOnly goal)
   {
      double minDistance = Double.POSITIVE_INFINITY;
      Point3DReadOnly pointTOReturn = null;
      for (Point3DReadOnly point : points)
      {
         double distance = point.distanceXY(goal);
         if (distance < minDistance)
         {
            minDistance = distance;
            pointTOReturn = point;
         }
      }

      return pointTOReturn;
   }



   @Disabled
   @Test
   public void testPartialShallowMaze()
   {
      testDataSet(DataSetName._20171114_135559_PartialShallowMaze);
   }

   @Disabled
   @Test
   public void testStairsUpDown()
   {
      testDataSet(DataSetName._20171215_214801_StairsUpDown);
   }

   @Disabled
   @Test
   public void testBodyPathPlannerEnvironment()
   {
      testDataSet(DataSetName._20171218_205120_BodyPathPlannerEnvironment);
   }

   private void testDataSet(DataSetName dataSetName)
   {
      DataSet dataSet = DataSetIOTools.loadDataSet(dataSetName);

      DefaultVisibilityGraphParameters parameters = new DefaultVisibilityGraphParameters();
      parameters.setPerformPostProcessingNodeShifting(true);

      PlanarRegionsList planarRegionsList = dataSet.getPlanarRegionsList();

      // test aligned with the edge of the wall, requiring slight offset
      Point3D start = dataSet.getPlannerInput().getStartPosition();
      Point3D goal = dataSet.getPlannerInput().getGoalPosition();
      Quaternion startOrientation = new Quaternion();
      Quaternion goalOrientation = new Quaternion();
      if (dataSet.getPlannerInput().hasStartOrientation())
         startOrientation.setToYawOrientation(dataSet.getPlannerInput().getStartYaw());
      if (dataSet.getPlannerInput().hasGoalOrientation())
         goalOrientation.setToYawOrientation(dataSet.getPlannerInput().getGoalYaw());

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);

      BodyPathPostProcessor postProcessor = new ObstacleAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      NavigableRegionsManager navigableRegionsManagerNoProcessor = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList());
      navigableRegionsManagerNoProcessor.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);
      List<Point3DReadOnly> pathNotProcessed = navigableRegionsManagerNoProcessor.calculateBodyPath(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator.computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution(),
                                                                                           startOrientation, goalOrientation);
      List<? extends Pose3DReadOnly> posePathNotProcessed = orientationCalculator.computePosesFromPath(pathNotProcessed, navigableRegionsManagerNoProcessor.getVisibilityMapSolution(),
                                                                                           startOrientation, goalOrientation);

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList(), navigableRegionsManager.getVisibilityMapSolution());
//         visualize(posePathNotProcessed, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList(), navigableRegionsManager.getVisibilityMapSolution());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());

   }

   @Test
   public void testGoingAroundACorner()
   {
      VisibilityGraphsParametersReadOnly parameters = createVisibilityGraphParametersForTest();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(TestEnvironmentTools.createCornerEnvironment());

      // test straight shot, initially going to one of the nodes
      Point3D start = new Point3D(0.0, 0.0, 0.0);
      Point3D goal = new Point3D(6.0, 30.0, 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      BodyPathPostProcessor postProcessor = new ObstacleAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPathWithOcclusions(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator.computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution(),
                                                                                           new Quaternion(), new Quaternion());

      /*
      if (visualize)
      {
         visualize(path, planarRegionsList, start, goal);
      }
      */

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList(), navigableRegionsManager.getVisibilityMapSolution());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   @Test
   public void testGoingAroundAU()
   {
      VisibilityGraphsParametersBasics parameters = new DefaultVisibilityGraphParameters();
      parameters.setPerformPostProcessingNodeShifting(true);
      parameters.setIntroduceMidpointsInPostProcessing(false);
      parameters.setComputeOrientationsToAvoidObstacles(false);

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(TestEnvironmentTools.createUCornerEnvironment());

      // test straight shot, initially going to one of the nodes
      Point3D start = new Point3D(-3.0, 0.0, 0.0);
      Point3D goal = new Point3D(-3.0, 10.0, 0.0);

      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      BodyPathPostProcessor postProcessor = new ObstacleAvoidanceProcessor(parameters);
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), postProcessor);
      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPathWithOcclusions(start, goal);
      List<? extends Pose3DReadOnly> posePath = orientationCalculator.computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution(),
                                                                                           new Quaternion(), new Quaternion());

      /*
      if (visualize)
      {
         visualize(path, planarRegionsList, start, goal);
      }
      */

      if (visualize)
      {
         visualize(posePath, parameters, planarRegionsList, start, goal, navigableRegionsManager.getNavigableRegionsList(), navigableRegionsManager.getVisibilityMapSolution());
      }

      checkPath(posePath, start, goal, parameters, planarRegionsList, navigableRegionsManager.getNavigableRegionsList());
   }

   private static void checkPath(List<? extends Pose3DReadOnly> path, Point3DReadOnly start, Point3DReadOnly goal, VisibilityGraphsParametersReadOnly parameters,
                                 PlanarRegionsList planarRegionsList, List<VisibilityMapWithNavigableRegion> navigableRegionsList)
   {
      checkPath(path, start, goal, parameters, planarRegionsList, navigableRegionsList, proximityEpsilon);
   }

   private static void checkPath(List<? extends Pose3DReadOnly> path, Point3DReadOnly start, Point3DReadOnly goal, VisibilityGraphsParametersReadOnly parameters,
                                 PlanarRegionsList planarRegionsList, List<VisibilityMapWithNavigableRegion> navigableRegionsList, double proximityEpsilon)
   {
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList(), null);

      navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());
      List<Point3DReadOnly> originalPath = navigableRegionsManager.calculateBodyPath(start, goal);

      String errorMessages = "";


      int numberOfPoints = path.size();
//      if (numberOfPoints < originalPath.size())
//         errorMessages += fail("number of points is not what was expected.");

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals("Did not start at the desired location.", start, path.get(0).getPosition(), epsilon);
//      EuclidCoreTestTools.assertPoint3DGeometricallyEquals("Did not end at the desired location.", goal, path.get(numberOfPoints - 1).getPosition(), epsilon);

      for (Pose3DReadOnly point : path)
      {
         assertFalse(point.containsNaN());
      }

      WaypointDefinedBodyPathPlanHolder calculatedPath = new WaypointDefinedBodyPathPlanHolder();
      calculatedPath.setPoseWaypoints(path);

      WaypointDefinedBodyPathPlanHolder expectedPathNoAvoidance = new WaypointDefinedBodyPathPlanHolder();
      expectedPathNoAvoidance.setWaypoints(originalPath);

      double distanceAlongExpectedPath = 0.0;

      Ellipsoid3D walkerShape = new Ellipsoid3D();
      walkerShape.getRadii().set(walkerRadii);
      List<Point3D> collisions = new ArrayList<>();


      for (double alpha = 0.05; alpha < 1.0; alpha += 0.001)
      {
         Pose3D expectedPose = new Pose3D();
         Pose3D actualPose = new Pose3D();

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
               ExtrusionHull clusterPolygon = obstacleCluster.getNonNavigableExtrusionsInWorld2D();
               Point2D closestPointOnCluster = new Point2D();
               double distanceToCluster = VisibilityTools.distanceToCluster(new Point2D(actualPose.getPosition()), clusterPolygon, closestPointOnCluster, null);
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

         String newErrorMessages = walkerCollisionChecks(walkerShape, planarRegionsList, collisions, proximityEpsilon);
         errorMessages += newErrorMessages;
         if (!newErrorMessages.isEmpty() && visualize)
            messager.submitMessage(UIVisibilityGraphsTopics.WalkerCollisionLocations, collisions);


         distanceToObstacles += parameters.getObstacleExtrusionDistance();

         if (visualize && distanceToObstacles < preferredObstacleExtrusionDistance - proximityEpsilon)
         {
            Point3DReadOnly collision = PlanarRegionTools.projectPointToPlanesVertically(new Point3D(closestPointOverall), planarRegionsList);
            collisions.add(new Point3D(collision));
            messager.submitMessage(UIVisibilityGraphsTopics.WalkerCollisionLocations, collisions);

         }

         if (distanceToObstacles < preferredObstacleExtrusionDistance - proximityEpsilon)
            errorMessages += fail("Was too close to an obstacle.");
      }

      if (!errorMessages.isEmpty() && !visualize)
         Assert.assertTrue(errorMessages, false);
      else
         LogTools.info(errorMessages);
   }

   private static String walkerCollisionChecks(Ellipsoid3D walkerShapeWorld, PlanarRegionsList planarRegionsList, List<Point3D> collisionsToPack, double proximityEpsilon)
   {
      String errorMessages = "";
      walkerShapeWorld = new Ellipsoid3D(walkerShapeWorld); // Make a copy to ensure we are not modifying the argument

      for (PlanarRegion planarRegion : planarRegionsList.getPlanarRegionsAsList())
      {
         Point3D walkerPosition3D = new Point3D(walkerShapeWorld.getPosition());

         Plane3D plane = planarRegion.getPlane();
         Point3DBasics closestPoint = plane.orthogonalProjectionCopy(walkerPosition3D);

         if (!walkerShapeWorld.isPointInside(closestPoint))
            continue; // Not even close to the region plane, let's keep going.

         Ellipsoid3D walkerShapeLocal = new Ellipsoid3D(walkerShapeWorld);
         planarRegion.transformFromWorldToLocal(walkerShapeLocal);
         walkerPosition3D.set(walkerShapeLocal.getPosition());
         Point2D walkerPosition2D = new Point2D(walkerPosition3D);

         if (planarRegion.getNumberOfConvexPolygons() == 0)
         {
            List<Point2DReadOnly> concaveHullVertices = new ArrayList<>(planarRegion.getConcaveHull());
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

   private static void visualize(List<? extends Pose3DReadOnly> path, VisibilityGraphsParametersReadOnly parameters, PlanarRegionsList planarRegionsList,
                                 Point3D start, Point3D goal, List<VisibilityMapWithNavigableRegion> navigableRegions, VisibilityMapSolution mapSolution)
   {
      visualize(path, parameters, planarRegionsList, start, goal, navigableRegions, mapSolution, proximityEpsilon);
   }

   private static void visualize(List<? extends Pose3DReadOnly> path, VisibilityGraphsParametersReadOnly parameters, PlanarRegionsList planarRegionsList,
                                 Point3D start, Point3D goal, List<VisibilityMapWithNavigableRegion> navigableRegions, VisibilityMapSolution mapSolution,
                                 double proximityEpsilon)
   {
      Random random = new Random(324);
      planarRegionsList.getPlanarRegionsAsList().forEach(region -> region.setRegionId(random.nextInt()));

      List<Point3DReadOnly> pathPoints = path.stream().map(pose -> new Point3D(pose.getPosition())).collect(Collectors.toList());
      visualizerApplication.submitPlanarRegionsListToVisualizer(planarRegionsList);
      visualizerApplication.submitGoalToVisualizer(goal);
      visualizerApplication.submitStartToVisualizer(start);
      visualizerApplication.submitNavigableRegionsToVisualizer(navigableRegions);
      visualizerApplication.submitVisibilityGraphSolutionToVisualizer(mapSolution);
      messager.submitMessage(UIVisibilityGraphsTopics.BodyPathData, pathPoints);

      while (true)
      {
         checkPath(path, start, goal, parameters, planarRegionsList, navigableRegions, proximityEpsilon);
      }
   }

   private VisibilityGraphsParametersBasics createVisibilityGraphParametersForTest()
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
//      parameters.setClusterResolution(0.501);
      parameters.setIntroduceMidpointsInPostProcessing(true);
      parameters.setPerformPostProcessingNodeShifting(true);
      parameters.setComputeOrientationsToAvoidObstacles(true);

      return parameters;
   }

}
