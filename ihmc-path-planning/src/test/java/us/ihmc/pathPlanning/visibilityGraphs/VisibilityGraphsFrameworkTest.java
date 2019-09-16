package us.ihmc.pathPlanning.visibilityGraphs;

import javafx.util.Pair;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.primitives.Ellipsoid3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.PlannerInput;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.NavigableRegionFilter;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.PlanarRegionFilter;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.PathOrientationCalculator;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullDecomposition;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.SpiralBasedAlgorithm;

import java.util.*;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.stream.Collectors;

public class VisibilityGraphsFrameworkTest
{
   // Threshold used to assert that the body path starts and ends where we asked it to.
   private static final double START_GOAL_EPSILON = 1.0e-1;

   // Whether to start the UI or not.
   private static boolean VISUALIZE = false;

   // Whether to fully expand the visibility graph or have it do efficient lazy evaluation.
   private static boolean fullyExpandVisibilityGraph = false;

   private static final int maxPointsInRegion = 25000;
   private static final double walkerTotalTime = 60.0;


   // For enabling helpful prints.
   private static boolean DEBUG = true;

   private static VisibilityGraphsTestVisualizerApplication visualizerApplication = null;
   // Because we use JavaFX, there will be two instance of VisibilityGraphsFrameworkTest, one for running the test and one starting the ui. The messager has to be static so both the ui and test use the same instance.
   private static JavaFXMessager messager = null;

   // The following are used for collision checks.
   private static final double walkerOffsetHeight = 0.75;
   private static final Vector3D walkerRadii = new Vector3D(0.25, 0.25, 0.5);
   protected static final double walkerMarchingSpeed = 0.2;
   private static final double lidarObserverHeight = 1.25;

   // For the occlusion test
   private static final int rays = 7500;
   private static final double rayLengthSquared = MathTools.square(5.0);

   private final ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
   private final PolygonizerParameters polygonizerParameters = new PolygonizerParameters();

   private VisibilityGraphsParametersReadOnly parameters;

   private static VisibilityGraphsParametersReadOnly createTestParameters()
   {
      VisibilityGraphsParametersBasics parameters = new DefaultVisibilityGraphParameters();
      parameters.setNormalZThresholdForAccessibleRegions(Math.cos(Math.toRadians(30.0)));
      return parameters;
   }

   @BeforeEach
   public void setup()
   {
      VISUALIZE = VISUALIZE && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
      DEBUG = (VISUALIZE || (DEBUG && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer()));

      if (VISUALIZE)
      {
         visualizerApplication = new VisibilityGraphsTestVisualizerApplication();
         visualizerApplication.startOnAThread();

         messager = visualizerApplication.getMessager();
      }

      parameters = createTestParameters();
   }

   @AfterEach
   public void tearDown() throws Exception
   {
      if (VISUALIZE)
      {
         visualizerApplication.stop();
         visualizerApplication = null;
         messager = null;
      }

      parameters = null;
   }

   @Test
   public void testDatasetsWithoutOcclusion()
   {
      if (VISUALIZE)
      {
         messager.submitMessage(UIVisibilityGraphsTopics.EnableWalkerAnimation, true);
         messager.submitMessage(UIVisibilityGraphsTopics.WalkerOffsetHeight, walkerOffsetHeight);
         messager.submitMessage(UIVisibilityGraphsTopics.WalkerSize, walkerRadii);
      }
      runAssertionsOnAllDatasets(dataset -> runAssertionsWithoutOcclusion(dataset), false);
   }

   @Test
   public void testDatasetsNoOcclusionSimulateDynamicReplanning()
   {
      if (VISUALIZE)
      {
         messager.submitMessage(UIVisibilityGraphsTopics.EnableWalkerAnimation, true);
         messager.submitMessage(UIVisibilityGraphsTopics.WalkerOffsetHeight, walkerOffsetHeight);
         messager.submitMessage(UIVisibilityGraphsTopics.WalkerSize, walkerRadii);
      }
      runAssertionsOnAllDatasets(dataset -> runAssertionsSimulateDynamicReplanning(dataset, walkerMarchingSpeed, 1000, false), false);
   }

   //TODO: Fix and make this pass.
   @Disabled("Occlusion planning needs to be implemented better.")
   @Test
   public void testDatasetsSimulateOcclusionAndDynamicReplanning()
   {
      if (VISUALIZE)
      {
         messager.submitMessage(UIVisibilityGraphsTopics.EnableWalkerAnimation, true);
         messager.submitMessage(UIVisibilityGraphsTopics.WalkerOffsetHeight, walkerOffsetHeight);
         messager.submitMessage(UIVisibilityGraphsTopics.WalkerSize, walkerRadii);
      }
      runAssertionsOnAllDatasets(dataset -> runAssertionsSimulateDynamicReplanning(dataset, walkerMarchingSpeed, 1000000000, true), true);
   }

   private void runAssertionsOnAllDatasets(Function<DataSet, String> dataSetTester, boolean testWithOcclusions)
   {
      Predicate<DataSet> dataSetFilter = dataSet ->
      {
         if(!dataSet.hasPlannerInput())
            return false;
         else if (testWithOcclusions && dataSet.getPlannerInput().getVisGraphCanRunWithOcclusion())
            return false;
         else
            return dataSet.getPlannerInput().getVisGraphIsTestable();
      };
      List<DataSet> allDatasets = DataSetIOTools.loadDataSets(dataSetFilter);

      if (DEBUG)
      {
         LogTools.info("Unit test files found: " + allDatasets.size());
         for (int i = 0; i < allDatasets.size(); i++)
            System.out.println("\t" + allDatasets.get(i).getName());
      }

      AtomicReference<Boolean> previousDatasetRequested = null;
      AtomicReference<Boolean> reloadDatasetRequested = null;
      AtomicReference<Boolean> nextDatasetRequested = null;
      AtomicReference<String> requestedDatasetPathReference = null;

      if (VISUALIZE)
      {
         List<String> allDatasetNames = allDatasets.stream().map(DataSet::getName).collect(Collectors.toList());
         messager.submitMessage(UIVisibilityGraphsTopics.AllDatasetPaths, allDatasetNames);

         nextDatasetRequested = messager.createInput(UIVisibilityGraphsTopics.NextDatasetRequest, false);
         reloadDatasetRequested = messager.createInput(UIVisibilityGraphsTopics.ReloadDatasetRequest, false);
         previousDatasetRequested = messager.createInput(UIVisibilityGraphsTopics.PreviousDatasetRequest, false);
         requestedDatasetPathReference = messager.createInput(UIVisibilityGraphsTopics.CurrentDatasetPath, null);
      }

      int numberOfFailingDatasets = 0;
      String errorMessages = "";

      int currentDatasetIndex = 0;
      if (allDatasets.isEmpty())
         Assert.fail("Did not find any datasets to test.");

      // Randomizing the regionIds so the viz is better
      Random random = new Random(324);
      allDatasets.stream().map(DataSet::getPlanarRegionsList).map(PlanarRegionsList::getPlanarRegionsAsList)
                 .forEach(regionsList -> regionsList.forEach(region -> region.setRegionId(random.nextInt())));

      DataSet dataset = allDatasets.get(currentDatasetIndex);

      while (dataset != null)
      {
         if (VISUALIZE)
         {
            messager.submitMessage(UIVisibilityGraphsTopics.GlobalReset, true);
            messager.submitMessage(UIVisibilityGraphsTopics.CurrentDatasetPath, dataset.getName());
         }

         if (DEBUG)
         {
            LogTools.info("Processing file: " + dataset.getName());
         }

         String errorMessagesForCurrentFile = dataSetTester.apply(dataset);
         if (!errorMessagesForCurrentFile.isEmpty())
            numberOfFailingDatasets++;
         errorMessages += errorMessagesForCurrentFile;


         if (DEBUG)
         {
            LogTools.info("Finished processing file: " + dataset.getName());
         }

         if (VISUALIZE)
         {
            messager.submitMessage(UIVisibilityGraphsTopics.NextDatasetRequest, false);
            messager.submitMessage(UIVisibilityGraphsTopics.ReloadDatasetRequest, false);
            messager.submitMessage(UIVisibilityGraphsTopics.PreviousDatasetRequest, false);

            while (!nextDatasetRequested.get() && !reloadDatasetRequested.get() && !previousDatasetRequested.get()
                  && dataset.getName().equals(requestedDatasetPathReference.get()))
            {
               if (!messager.isMessagerOpen())
                  return; // The ui has been closed

               ThreadTools.sleep(200);
            }

            if (reloadDatasetRequested.get())
            {
               continue;
            }
            else if (nextDatasetRequested.get() && currentDatasetIndex < allDatasets.size() - 1)
            {
               currentDatasetIndex++;
               dataset = allDatasets.get(currentDatasetIndex);
            }
            else if (previousDatasetRequested.get() && currentDatasetIndex > 0)
            {
               currentDatasetIndex--;
               dataset = allDatasets.get(currentDatasetIndex);
            }
            else if (requestedDatasetPathReference.get() != null)
            {
               String path = requestedDatasetPathReference.get();
               DataSet requestedDataset = allDatasets.stream().filter(d -> d.getName().equals(path)).findFirst().orElse(null);
               if (requestedDataset == null)
               {
                  LogTools.error("Could not find the requested dataset with name: " + path);
               }
               else
               {
                  currentDatasetIndex = allDatasets.indexOf(requestedDataset);
                  dataset = requestedDataset;
               }
            }
            else
            {
               dataset = null;
            }
         }
         else
         {
            currentDatasetIndex++;
            if (currentDatasetIndex < allDatasets.size())
               dataset = allDatasets.get(currentDatasetIndex);
            else
               dataset = null;
         }



         ThreadTools.sleep(100); // Apparently need to give some time for the prints to appear in the right order.
      }

      Assert.assertTrue("Number of failing datasets: " + numberOfFailingDatasets + " out of " + allDatasets.size() + ". Errors:" + errorMessages,
                        errorMessages.isEmpty());
   }

   private void runAssertionsOnDataset(Function<DataSet, String> dataSetTester, String datasetname)
   {
      List<DataSet> allDatasets = DataSetIOTools.loadDataSets();

      if (DEBUG)
      {
         LogTools.info("Unit test files found: " + allDatasets.size());
      }

      if (VISUALIZE)
      {
         List<String> allDatasetNames = allDatasets.stream().map(DataSet::getName).collect(Collectors.toList());
         messager.submitMessage(UIVisibilityGraphsTopics.AllDatasetPaths, allDatasetNames);

      }

      if (allDatasets.isEmpty())
         Assert.fail("Did not find any datasets to test.");

      // Randomizing the regionIds so the viz is better
      Random random = new Random(324);
      allDatasets.stream().map(DataSet::getPlanarRegionsList).map(PlanarRegionsList::getPlanarRegionsAsList)
                 .forEach(regionsList -> regionsList.forEach(region -> region.setRegionId(random.nextInt())));

      DataSet dataset = allDatasets.stream().filter(d -> d.getName().equals(datasetname)).findFirst().orElse(null);

      if (VISUALIZE)
      {
         messager.submitMessage(UIVisibilityGraphsTopics.GlobalReset, true);
         messager.submitMessage(UIVisibilityGraphsTopics.CurrentDatasetPath, dataset.getName());
      }

      if (DEBUG)
      {
         LogTools.info("Processing file: " + dataset.getName());
      }

      String errorMessages = dataSetTester.apply(dataset);

      Assert.assertTrue("Errors: " + errorMessages, errorMessages.isEmpty());
      LogTools.info("Finished testing.");
      ThreadTools.sleepForever(); // Apparently need to give some time for the prints to appear in the right order.
   }

   private String runAssertionsWithoutOcclusion(DataSet dataset)
   {
      String datasetName = dataset.getName();

      PlanarRegionsList planarRegionsList = dataset.getPlanarRegionsList();

      PlannerInput plannerInput = dataset.getPlannerInput();
      Point3D start = plannerInput.getStartPosition();
      Point3D goal = plannerInput.getGoalPosition();

      if (VISUALIZE)
      {
         visualizerApplication.submitPlanarRegionsListToVisualizer(planarRegionsList);
         visualizerApplication.submitStartToVisualizer(start);
         visualizerApplication.submitGoalToVisualizer(goal);
      }

      String errorMessages = calculateAndTestVizGraphsBodyPath(datasetName, start, goal, planarRegionsList);

      return addPrefixToErrorMessages(datasetName, errorMessages);
   }

   private String runAssertionsWithOcclusions(DataSet dataset)
   {
      String datasetName = dataset.getName();

      PlanarRegionsList planarRegionsList = dataset.getPlanarRegionsList();

      HashMap<PlanarRegion, List<Point3D>> pointsInRegions = new HashMap<>();
      for (PlanarRegion planarRegion : planarRegionsList.getPlanarRegionsAsList())
         pointsInRegions.put(planarRegion, new ArrayList<>());


      PlannerInput plannerInput = dataset.getPlannerInput();
      Point3D start = plannerInput.getStartPosition();
      Point3D goal = plannerInput.getGoalPosition();

      Point3D observer = new Point3D();
      observer.set(start);
      observer.addZ(lidarObserverHeight);
      Pair<PlanarRegionsList, List<Point3D>> result = createVisibleRegions(planarRegionsList, pointsInRegions, observer);

      PlanarRegionsList visibleRegions = result.getKey();

      if (VISUALIZE)
      {
         visualizerApplication.submitPlanarRegionsListToVisualizer(visibleRegions);

         visualizerApplication.submitShadowPlanarRegionsListToVisualizer(planarRegionsList);
         visualizerApplication.submitStartToVisualizer(start);
         visualizerApplication.submitGoalToVisualizer(goal);
      }

      String errorMessages = calculateAndTestVizGraphsBodyPath(datasetName, start, goal, visibleRegions);

      return addPrefixToErrorMessages(datasetName, errorMessages);
   }

   private String runAssertionsSimulateDynamicReplanning(DataSet dataset, double walkerSpeed, long maxSolveTimeInMilliseconds,
                                                         boolean simulateOcclusions)
   {
      String datasetName = dataset.getName();

      PlanarRegionsList planarRegionsList = dataset.getPlanarRegionsList();
      HashMap<PlanarRegion, List<Point3D>> pointsInRegions = new HashMap<>();
      for (PlanarRegion planarRegion : planarRegionsList.getPlanarRegionsAsList())
         pointsInRegions.put(planarRegion, new ArrayList<>());

      PlannerInput plannerInput = dataset.getPlannerInput();
      Point3D start = plannerInput.getStartPosition();
      Point3D goal = plannerInput.getGoalPosition();
      AtomicReference<Boolean> stopWalkerRequest = null;

      if (VISUALIZE)
      {
         stopWalkerRequest = messager.createInput(UIVisibilityGraphsTopics.StopWalker, false);
//         if (simulateOcclusions)
//            visualizerApplication.submitShadowPlanarRegionsListToVisualizer(planarRegionsList);
         if (!simulateOcclusions)
            visualizerApplication.submitPlanarRegionsListToVisualizer(planarRegionsList);

         visualizerApplication.submitStartToVisualizer(start);
         visualizerApplication.submitGoalToVisualizer(goal);
      }

      String errorMessages = "";

      List<Pose3DReadOnly> latestBodyPath = new ArrayList<>();

      Point3D walkerPosition = new Point3D(start);

      long totalStartTime = System.currentTimeMillis();

      while (!walkerPosition.geometricallyEquals(goal, 1.0e-2))
      {
         PlanarRegionsList visibleRegions = planarRegionsList;

         if (simulateOcclusions)
         {
            Point3D observer = new Point3D();
            observer.set(walkerPosition);
            observer.addZ(lidarObserverHeight);
            Pair<PlanarRegionsList, List<Point3D>> result = createVisibleRegions(planarRegionsList, pointsInRegions, observer);
            visibleRegions = result.getKey();
         }

         if (VISUALIZE)
         {
            if (simulateOcclusions)
               visualizerApplication.submitPlanarRegionsListToVisualizer(visibleRegions);
         }

         long startTime = System.currentTimeMillis();
         errorMessages += calculateAndTestVizGraphsBodyPath(datasetName, walkerPosition, goal, visibleRegions, latestBodyPath, simulateOcclusions);
         long endTime = System.currentTimeMillis();
         if (endTime - startTime > maxSolveTimeInMilliseconds)
            errorMessages += fail(datasetName, "Took too long to compute a new body path.");

         if (endTime - totalStartTime > Conversions.secondsToMilliseconds(walkerTotalTime))
            errorMessages += fail(datasetName, "Took too long to make it through the body path. Made it to " + walkerPosition + ", while the goal was " + goal);

         if (!errorMessages.isEmpty())
            return addPrefixToErrorMessages(datasetName, errorMessages);

         if (VISUALIZE)
         {
            messager.submitMessage(UIVisibilityGraphsTopics.WalkerPosition, walkerPosition);
         }

         if (stopWalkerRequest != null && stopWalkerRequest.get())
         {
            messager.submitMessage(UIVisibilityGraphsTopics.StopWalker, false);
            break;
         }

         walkerPosition.set(travelAlongBodyPath(walkerSpeed, latestBodyPath));

         if (VISUALIZE)
         {
            if (!messager.isMessagerOpen())
               return addPrefixToErrorMessages(datasetName, errorMessages); // The ui has been closed
         }
      }

      return addPrefixToErrorMessages(datasetName, errorMessages);
   }

   private static Point3DReadOnly travelAlongBodyPath(double distanceToTravel, List<Pose3DReadOnly> bodyPath)
   {
      Point3D initialPosition = new Point3D(bodyPath.get(0).getPosition());

      Point3D positionWithShift = new Point3D();

      for (int i = 0; i < bodyPath.size() - 1; i++)
      {
         LineSegment3D segment = new LineSegment3D(bodyPath.get(i).getPosition(), bodyPath.get(i + 1).getPosition());
         if (segment.length() < 5e-3)
            continue;

         if (xyDistance(segment, initialPosition) < 1.0e-2)
         {
            Vector3DBasics segmentDirection = segment.getDirection(true);
            positionWithShift.scaleAdd(distanceToTravel, segmentDirection, initialPosition);

            if (xyDistance(segment, positionWithShift) < 1.0e-2)
            {
               initialPosition = new Point3D(positionWithShift);
               break;
            }
            else
            {
               distanceToTravel -= initialPosition.distanceXY(segment.getSecondEndpoint());
               initialPosition = new Point3D(segment.getSecondEndpoint());
            }
         }
      }

      if (initialPosition.getZ() > 2.0)
         return initialPosition;
      else
         return initialPosition;
   }

   private static double xyDistance(LineSegment3DReadOnly segment, Point3DReadOnly point)
   {
      Point3DReadOnly lineSegmentStart = segment.getFirstEndpoint();
      Point3DReadOnly lineSegmentEnd = segment.getSecondEndpoint();
      return EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(point.getX(), point.getY(), lineSegmentStart.getX(), lineSegmentStart.getY(), lineSegmentEnd.getX(), lineSegmentEnd.getY());
   }

   private String calculateAndTestVizGraphsBodyPath(String datasetName, Point3D start, Point3D goal, PlanarRegionsList planarRegionsList)
   {
      return calculateAndTestVizGraphsBodyPath(datasetName, start, goal, planarRegionsList, null);
   }

   private String calculateAndTestVizGraphsBodyPath(String datasetName, Point3D start, Point3D goal, PlanarRegionsList planarRegionsList,
                                                    List<Pose3DReadOnly> bodyPathToPack)
   {
      return calculateAndTestVizGraphsBodyPath(datasetName, start, goal, planarRegionsList, bodyPathToPack, false);
   }

   private String calculateAndTestVizGraphsBodyPath(String datasetName, Point3D start, Point3D goal, PlanarRegionsList planarRegionsList,
                                                    List<Pose3DReadOnly> bodyPathToPack, boolean simulateOcclusions)
   {
      NavigableRegionsManager manager = new NavigableRegionsManager(parameters);
      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
      manager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<? extends Pose3DReadOnly> path = null;
      List<Point3DReadOnly> pathPoints = null;

      try
      {
         pathPoints = manager.calculateBodyPath(start, goal, fullyExpandVisibilityGraph);
         path = orientationCalculator.computePosesFromPath(pathPoints, manager.getVisibilityMapSolution());
//         path = manager.calculateBodyPathWithOcclusions(start, goal,);
      }
      catch (Exception e)
      {
         e.printStackTrace();
         ThreadTools.sleep(100); // Give some time to the exception to print.
      }

      if (VISUALIZE)
      {
         if (path != null)
         {
            messager.submitMessage(UIVisibilityGraphsTopics.BodyPathData, pathPoints);
         }

         visualizerApplication.submitVisibilityGraphSolutionToVisualizer(manager.getVisibilityMapSolution());
      }

      String errorMessages = basicBodyPathSanityChecks(datasetName, start, goal, path, !simulateOcclusions);

      if (!errorMessages.isEmpty())
         return errorMessages; // Cannot test anything else when path does not pass the basic sanity checks.

      if (bodyPathToPack != null)
      {
         bodyPathToPack.clear();
         bodyPathToPack.addAll(path);
      }

      // "Walk" along the body path and assert that the walker does not go through any region.
      int currentSegmentIndex = 0;
      Point3DReadOnly pathStart = path.get(0).getPosition();
      Point3DReadOnly pathEnd = path.get(path.size() - 1).getPosition();
      Point3D walkerCurrentPosition = new Point3D(pathStart);
      List<Point3D> collisions = new ArrayList<>();
      Ellipsoid3D walkerShape = new Ellipsoid3D();
      walkerShape.setRadii(walkerRadii);

      Point3D walkerBody3D = new Point3D(walkerCurrentPosition);
      walkerBody3D.addZ(walkerOffsetHeight);
      walkerShape.getPosition().set(walkerBody3D);

      errorMessages += walkerCollisionChecks(datasetName, walkerShape, planarRegionsList, collisions, parameters, true);

      while (!walkerCurrentPosition.geometricallyEquals(pathEnd, 1.0e-2))
      {
         walkerBody3D = new Point3D(walkerCurrentPosition);
         walkerBody3D.addZ(walkerOffsetHeight);
         walkerShape.getPosition().set(walkerBody3D);

         errorMessages += walkerCollisionChecks(datasetName, walkerShape, planarRegionsList, collisions, parameters, !simulateOcclusions);

         //         walkerCurrentPosition.set(travelAlongBodyPath(walkerMarchingSpeed, walkerCurrentPosition, path));
         Point3DReadOnly segmentStart = path.get(currentSegmentIndex).getPosition();
         Point3DReadOnly segmentEnd = path.get(currentSegmentIndex + 1).getPosition();


         Vector3D segmentDirection = new Vector3D();
         segmentDirection.sub(segmentEnd, segmentStart);
         segmentDirection.normalize();
         if (segmentDirection.containsNaN() || segmentDirection.length() < 1e-2)
         {
            currentSegmentIndex++;
            continue;
         }
         walkerCurrentPosition.scaleAdd(walkerMarchingSpeed, segmentDirection, walkerCurrentPosition);
         if (segmentStart.distance(segmentEnd) < segmentStart.distance(walkerCurrentPosition))
         {
            walkerCurrentPosition.set(segmentEnd);
            currentSegmentIndex++;
         }
      }

      if (VISUALIZE)
      {
         messager.submitMessage(UIVisibilityGraphsTopics.WalkerCollisionLocations, collisions);
      }

      return errorMessages;
   }

   private String walkerCollisionChecks(String datasetName, Ellipsoid3D walkerShapeWorld, PlanarRegionsList planarRegionsList, List<Point3D> collisionsToPack,
                                        VisibilityGraphsParametersReadOnly visibilityGraphsParameters, boolean checkForShapeCollision)
   {
      String errorMessages = "";
      walkerShapeWorld = new Ellipsoid3D(walkerShapeWorld); // Make a copy to ensure we are not modifying the argument

      NavigableRegionFilter navigableFilter = visibilityGraphsParameters.getNavigableRegionFilter();
      PlanarRegionFilter planarRegionFilter = visibilityGraphsParameters.getPlanarRegionFilter();
      List<PlanarRegion> regionsToCheck = planarRegionsList.getPlanarRegionsAsList().stream().filter(query -> navigableFilter.isPlanarRegionNavigable(query, planarRegionsList.getPlanarRegionsAsList())).collect(Collectors.toList());
      regionsToCheck = regionsToCheck.stream().filter(planarRegionFilter::isPlanarRegionRelevant).collect(Collectors.toList());

      for (PlanarRegion planarRegion : regionsToCheck)
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
               if (convexPolygon.isPointInside(walkerPosition2D))
               {
                  collisionsToPack.add(walkerPosition3D);
                  errorMessages += fail(datasetName, "Body path is going through a region."); // TODO figure out the proper intersection
                  break;
               }
               else
               {
                  throw new RuntimeException("Not sure what went wrong to here.");
               }
            }
            closestPoint = new Point3D(closestPoint2D);

            if (walkerShapeLocal.isPointInside(closestPoint) && checkForShapeCollision)
            {
               Point2DBasics intersectionLocal = closestPoint2D;
               Point3D intersectionWorld = new Point3D(intersectionLocal);
               planarRegion.transformFromLocalToWorld(intersectionWorld);
               errorMessages += fail(datasetName, "Walker is going through a region at: " + intersectionWorld);
               collisionsToPack.add(intersectionWorld);
            }
         }
      }
      return errorMessages;
   }

   private String basicBodyPathSanityChecks(String datasetName, Point3DReadOnly start, Point3DReadOnly goal, List<? extends Pose3DReadOnly> path, boolean checkEnds)
   {
      String errorMessages = "";
      errorMessages += assertTrue(datasetName, "Path is null!", path != null);
      if (!errorMessages.isEmpty())
         return errorMessages; // Cannot test anything else when no path is returned.

      errorMessages += assertTrue(datasetName, "Path does not contain any waypoints", path.size() > 0);

      if (!errorMessages.isEmpty())
         return errorMessages; // Cannot test anything else when no path is returned.

      Point3DReadOnly pathEnd = path.get(path.size() - 1).getPosition();
      Point3DReadOnly pathStart = path.get(0).getPosition();

      Point2DReadOnly pathEnd2D = new Point2D(pathEnd);
      Point2DReadOnly pathStart2D = new Point2D(pathStart);

      Point2DReadOnly goal2D = new Point2D(goal);
      Point2DReadOnly start2D = new Point2D(start);

      if (checkEnds)
      {
               errorMessages += assertTrue(datasetName, "Body path does not end at desired goal position: desired = " + goal + ", actual = " + pathEnd,
                                           pathEnd2D.geometricallyEquals(goal2D, START_GOAL_EPSILON));
      }
      errorMessages += assertTrue(datasetName, "Body path does not start from desired start position: desired = " + start + ", actual = " + pathStart,
                                  pathStart2D.geometricallyEquals(start2D, START_GOAL_EPSILON));

      return errorMessages;
   }


   // TODO See if possible to support concave hulls instead of convex. It changes the shape of the regions quite some sometimes.
   private Pair<PlanarRegionsList, List<Point3D>> createVisibleRegions(PlanarRegionsList regions, HashMap<PlanarRegion, List<Point3D>> pointsInRegions,
                                                                       Point3D observer)
   {
      List<Point3D> rayImpactLocations = new ArrayList<>();
      Point3D[] pointsOnSphere = SpiralBasedAlgorithm.generatePointsOnSphere(observer, 1.0, rays);


      List<PlanarRegion> filteredRegions = PlanarRegionTools.filterPlanarRegionsWithBoundingCircle(new Point2D(observer), Math.sqrt(rayLengthSquared), regions.getPlanarRegionsAsList());

      for (int rayIndex = 0; rayIndex < rays; rayIndex++)
      {
         Point3D pointOnSphere = pointsOnSphere[rayIndex];
         Vector3D rayDirection = new Vector3D();
         rayDirection.sub(pointOnSphere, observer);
         ImmutablePair<Point3D, PlanarRegion> intersectionPair = PlanarRegionTools.intersectRegionsWithRay(filteredRegions, observer, rayDirection);
         if (intersectionPair == null || intersectionPair.getLeft().distanceSquared(observer) > rayLengthSquared)
         {
            continue;
         }

         Point3D intersection = intersectionPair.getLeft();
         PlanarRegion region = intersectionPair.getRight();

         rayImpactLocations.add(intersection);

         Point3D pointOnPlane = new Point3D(intersection);
         region.transformFromWorldToLocal(pointOnPlane);

         pointsInRegions.get(region).add(intersection);
      }

      for (PlanarRegion region : pointsInRegions.keySet())
      {
         List<Point3D> pointsInRegion = pointsInRegions.get(region);
         while (pointsInRegion.size() > maxPointsInRegion)
            pointsInRegion.remove(0);
      }


      List<PlanarRegionSegmentationRawData> segmentationRawData = new ArrayList<>();
      for (PlanarRegion originalRegion : pointsInRegions.keySet())
      {
         List<Point3D> points = pointsInRegions.get(originalRegion);
         Point3D center = new Point3D();
         originalRegion.getBoundingBox3dInWorld().getCenterPoint(center);
         PlanarRegionSegmentationRawData rawData = new PlanarRegionSegmentationRawData(originalRegion.getRegionId(), originalRegion.getNormal(),
                                                                                       center, points);
         segmentationRawData.add(rawData);
      }

      PlanarRegionsList visibleRegionsList = PlanarRegionPolygonizer.createPlanarRegionsList(segmentationRawData, concaveHullFactoryParameters, polygonizerParameters);

      return new Pair<>(visibleRegionsList, rayImpactLocations);
   }


   private static String addPrefixToErrorMessages(String datasetName, String errorMessages)
   {
      if (!errorMessages.isEmpty())
         return "\n" + datasetName + errorMessages;
      else
         return "";
   }

   private String fail(String datasetName, String message)
   {
      return assertTrue(datasetName, message, false);
   }

   private String assertTrue(String datasetName, String message, boolean condition)
   {
      if (VISUALIZE || DEBUG)
      {
         if (!condition)
            LogTools.error(datasetName + ": " + message);
      }
      return !condition ? "\n" + message : "";
   }

   public static void main(String[] args) throws Exception
   {
      VisibilityGraphsFrameworkTest test = new VisibilityGraphsFrameworkTest();
//      String dataSetName = "20171218_205120_BodyPathPlannerEnvironment";
      String dataSetName = "20171215_211034_DoorwayNoCeiling";
//      String dataSetName = "20171218_204953_FlatGroundWithWall";
//      String dataSetName = "20171215_220523_SteppingStones";
//      String dataSetName = "20171218_204917_FlatGround";
//      String dataSetName = "20171215_214730_CinderBlockField";
//      String dataSetName = "20001201_205050_TwoSquaresOneObstacle";
//      String dataSetName = "20171215_210811_DoorwayWithCeiling";

      test.setup();
      if (VISUALIZE)
      {
         messager.submitMessage(UIVisibilityGraphsTopics.EnableWalkerAnimation, false);
         messager.submitMessage(UIVisibilityGraphsTopics.WalkerOffsetHeight, walkerOffsetHeight);
         messager.submitMessage(UIVisibilityGraphsTopics.WalkerSize, walkerRadii);
      }
      test.runAssertionsOnDataset(dataset -> test.runAssertionsSimulateDynamicReplanning(dataset, walkerMarchingSpeed, 100000000, true), dataSetName);
//      test.runAssertionsOnDataset(dataset -> test.runAssertionsWithoutOcclusion(dataset), dataSetName);
      test.tearDown();

   }
}
