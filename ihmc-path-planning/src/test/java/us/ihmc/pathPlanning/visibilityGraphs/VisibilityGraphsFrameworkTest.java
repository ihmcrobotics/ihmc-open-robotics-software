package us.ihmc.pathPlanning.visibilityGraphs;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.stream.Collectors;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import javafx.util.Pair;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.primitives.Ellipsoid3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
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
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullDecomposition;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.SpiralBasedAlgorithm;

public class VisibilityGraphsFrameworkTest
{
   // Set that to MAX_VALUE when visualizing. Before pushing, it has to be reset to a reasonable value.
   private static final long TIMEOUT = 100000; // Long.MAX_VALUE; //
   // Threshold used to assert that the body path starts and ends where we asked it to.
   private static final double START_GOAL_EPSILON = 1.0e-2;

   // Whether to start the UI or not.
   private static boolean VISUALIZE = false;

   // Whether to fully expand the visibility graph or have it do efficient lazy evaluation.
   private static boolean fullyExpandVisibilityGraph = false;

   // For enabling helpful prints.
   private static boolean DEBUG = true;

   private static VisibilityGraphsTestVisualizerApplication visualizerApplication = null;
   // Because we use JavaFX, there will be two instance of VisibilityGraphsFrameworkTest, one for running the test and one starting the ui. The messager has to be static so both the ui and test use the same instance.
   private static JavaFXMessager messager = null;

   // The following are used for collision checks.
   private static final double walkerOffsetHeight = 0.75;
   private static final Vector3D walkerRadii = new Vector3D(0.25, 0.25, 0.5);
   private static final double walkerMarchingSpeed = 0.05;

   // For the occlusion test
   private static final int rays = 5000;
   private static final double rayLengthSquared = MathTools.square(5.0);

   private static VisibilityGraphsParametersReadOnly createTestParameters()
   {
      return new DefaultVisibilityGraphParameters()
      {
         @Override
         public double getNormalZThresholdForAccessibleRegions()
         {
            return Math.cos(Math.toRadians(30.0));
         }
      };
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
      runAssertionsOnAllDatasets(dataset -> runAssertionsWithoutOcclusion(dataset));
   }

   //TODO: Fix and make this pass.
   @Disabled("This needs to be fixed for when start and goal are invalid.")
   @Test
   public void testDatasetsNoOcclusionSimulateDynamicReplanning()
   {
      if (VISUALIZE)
      {
         messager.submitMessage(UIVisibilityGraphsTopics.EnableWalkerAnimation, true);
         messager.submitMessage(UIVisibilityGraphsTopics.WalkerOffsetHeight, walkerOffsetHeight);
         messager.submitMessage(UIVisibilityGraphsTopics.WalkerSize, walkerRadii);
      }
      runAssertionsOnAllDatasets(dataset -> runAssertionsSimulateDynamicReplanning(dataset, 0.20, 1000, false));
   }

   //TODO: Fix and make this pass.
   @Disabled("Occlusion planning needs to be implemented better.")
   @Test
   public void testDatasetsSimulateOcclusionAndDynamicReplanning()
   {
      if (VISUALIZE)
      {
         messager.submitMessage(UIVisibilityGraphsTopics.EnableWalkerAnimation, false);
      }
      runAssertionsOnAllDatasets(dataset -> runAssertionsSimulateDynamicReplanning(dataset, 0.20, 1000, true));
   }

   private void runAssertionsOnAllDatasets(Function<DataSet, String> dataSetTester)
   {
      Predicate<DataSet> dataSetFilter = dataSet ->
      {
         if(!dataSet.hasPlannerInput())
            return false;
         else
            return dataSet.getPlannerInput().getVisGraphIsTestable();
      };
      List<DataSet> allDatasets = DataSetIOTools.loadDataSets(dataSetFilter);

      if (DEBUG)
      {
         LogTools.info("Unit test files found: " + allDatasets.size());
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

   private String runAssertionsSimulateDynamicReplanning(DataSet dataset, double walkerSpeed, long maxSolveTimeInMilliseconds,
                                                         boolean simulateOcclusions)
   {
      String datasetName = dataset.getName();

      PlanarRegionsList planarRegionsList = dataset.getPlanarRegionsList();

      PlannerInput plannerInput = dataset.getPlannerInput();
      Point3D start = plannerInput.getStartPosition();
      Point3D goal = plannerInput.getGoalPosition();
      AtomicReference<Boolean> stopWalkerRequest = null;

      if (VISUALIZE)
      {
         stopWalkerRequest = messager.createInput(UIVisibilityGraphsTopics.StopWalker, false);
         if (simulateOcclusions)
            visualizerApplication.submitShadowPlanarRegionsListToVisualizer(planarRegionsList);
         else
            visualizerApplication.submitPlanarRegionsListToVisualizer(planarRegionsList);

         visualizerApplication.submitStartToVisualizer(start);
         visualizerApplication.submitGoalToVisualizer(goal);
      }

      String errorMessages = "";

      List<Point3DReadOnly> latestBodyPath = new ArrayList<>();

      Point3D walkerPosition = new Point3D(start);

      PlanarRegionsList knownRegions = new PlanarRegionsList();

      while (!walkerPosition.geometricallyEquals(goal, 1.0e-3))
      {
         PlanarRegionsList visibleRegions = planarRegionsList;

         if (simulateOcclusions)
         {
            Point3D observer = new Point3D();
            observer.set(walkerPosition);
            observer.addZ(0.8);
            Pair<PlanarRegionsList, List<Point3D>> result = createVisibleRegions(planarRegionsList, observer, knownRegions);
            knownRegions = result.getKey(); // For next iteration
            visibleRegions = result.getKey();
         }

         if (VISUALIZE)
         {
            if (simulateOcclusions)
               visualizerApplication.submitPlanarRegionsListToVisualizer(knownRegions);
         }

         long startTime = System.currentTimeMillis();
         errorMessages += calculateAndTestVizGraphsBodyPath(datasetName, walkerPosition, goal, visibleRegions, latestBodyPath);
         long endTime = System.currentTimeMillis();
         if (endTime - startTime > maxSolveTimeInMilliseconds)
            errorMessages += fail(datasetName, "Took too long to compute a new body path.");

         if (!errorMessages.isEmpty())
            return addPrefixToErrorMessages(datasetName, errorMessages);

         if (VISUALIZE)
         {
            messager.submitMessage(UIVisibilityGraphsTopics.WalkerPosition, new Point3D(walkerPosition));
         }

         if (stopWalkerRequest != null && stopWalkerRequest.get())
         {
            messager.submitMessage(UIVisibilityGraphsTopics.StopWalker, false);
            break;
         }

         walkerPosition.set(travelAlongBodyPath(walkerSpeed, walkerPosition, latestBodyPath));

         if (VISUALIZE)
         {
            if (!messager.isMessagerOpen())
               return addPrefixToErrorMessages(datasetName, errorMessages); // The ui has been closed
         }
      }

      return addPrefixToErrorMessages(datasetName, errorMessages);
   }

   private static Point3D travelAlongBodyPath(double distanceToTravel, Point3DReadOnly startingPosition, List<Point3DReadOnly> bodyPath)
   {
      Point3D newPosition = new Point3D();

      for (int i = 0; i < bodyPath.size() - 1; i++)
      {
         LineSegment3D segment = new LineSegment3D(bodyPath.get(i), bodyPath.get(i + 1));

         if (xyDistance(segment, startingPosition) < 1.0e-4)
         {
            Vector3DBasics segmentDirection = segment.getDirection(true);
            newPosition.scaleAdd(distanceToTravel, segmentDirection, startingPosition);

            if (segment.distance(newPosition) < 1.0e-4)
            {
               return newPosition;
            }
            else
            {
               distanceToTravel -= startingPosition.distance(segment.getSecondEndpoint());
               startingPosition = new Point3D(segment.getSecondEndpoint());
            }
         }
      }

      return new Point3D(startingPosition);
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
                                                    List<Point3DReadOnly> bodyPathToPack)
   {
      VisibilityGraphsParametersReadOnly parameters = createTestParameters();
      NavigableRegionsManager manager = new NavigableRegionsManager(parameters);
      manager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = null;

      try
      {
         path = manager.calculateBodyPath(start, goal, fullyExpandVisibilityGraph);
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
            messager.submitMessage(UIVisibilityGraphsTopics.BodyPathData, path);
         }

         visualizerApplication.submitVisibilityGraphSolutionToVisualizer(manager.getVisibilityMapSolution());
      }

      String errorMessages = basicBodyPathSanityChecks(datasetName, start, goal, path);

      if (!errorMessages.isEmpty())
         return errorMessages; // Cannot test anything else when path does not pass the basic sanity checks.

      if (bodyPathToPack != null)
      {
         bodyPathToPack.clear();
         bodyPathToPack.addAll(path);
      }

      // "Walk" along the body path and assert that the walker does not go through any region.
      int currentSegmentIndex = 0;
      Point3DReadOnly pathStart = path.get(0);
      Point3DReadOnly pathEnd = path.get(path.size() - 1);
      Point3D walkerCurrentPosition = new Point3D(pathStart);
      List<Point3D> collisions = new ArrayList<>();
      Ellipsoid3D walkerShape = new Ellipsoid3D();
      walkerShape.setRadii(walkerRadii);

      while (!walkerCurrentPosition.geometricallyEquals(pathEnd, 1.0e-3))
      {
         Point3D walkerBody3D = new Point3D(walkerCurrentPosition);
         walkerBody3D.addZ(walkerOffsetHeight);
         walkerShape.getPosition().set(walkerBody3D);

         errorMessages += walkerCollisionChecks(datasetName, walkerShape, planarRegionsList, collisions);

         Point3DReadOnly segmentStart = path.get(currentSegmentIndex);
         Point3DReadOnly segmentEnd = path.get(currentSegmentIndex + 1);
         Vector3D segmentDirection = new Vector3D();
         segmentDirection.sub(segmentEnd, segmentStart);
         segmentDirection.normalize();
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

   private String walkerCollisionChecks(String datasetName, Ellipsoid3D walkerShapeWorld, PlanarRegionsList planarRegionsList, List<Point3D> collisionsToPack)
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
               if (convexPolygon.isPointInside(walkerPosition2D))
               {
                  errorMessages += fail(datasetName, "Body path is going through a region."); // TODO figure out the proper intersection
                  break;
               }
               else
               {
                  throw new RuntimeException("Not usre what went wrong to here.");
               }
            }
            closestPoint = new Point3D(closestPoint2D);

            if (walkerShapeLocal.isPointInside(closestPoint))
            {
               Point2DBasics intersectionLocal = closestPoint2D;
               Point3D intersectionWorld = new Point3D(intersectionLocal);
               planarRegion.transformFromLocalToWorld(intersectionWorld);
               errorMessages += fail(datasetName, "Body path is going through a region at: " + intersectionWorld);
               collisionsToPack.add(intersectionWorld);
            }
         }
      }
      return errorMessages;
   }

   private String basicBodyPathSanityChecks(String datasetName, Point3DReadOnly start, Point3DReadOnly goal, List<? extends Point3DReadOnly> path)
   {
      String errorMessages = "";
      errorMessages += assertTrue(datasetName, "Path is null!", path != null);
      if (!errorMessages.isEmpty())
         return errorMessages; // Cannot test anything else when no path is returned.

      errorMessages += assertTrue(datasetName, "Path does not contain any waypoints", path.size() > 0);

      if (!errorMessages.isEmpty())
         return errorMessages; // Cannot test anything else when no path is returned.

      Point3DReadOnly pathEnd = path.get(path.size() - 1);
      Point3DReadOnly pathStart = path.get(0);

      Point2DReadOnly pathEnd2D = new Point2D(pathEnd);
      Point2DReadOnly pathStart2D = new Point2D(pathStart);

      Point2DReadOnly goal2D = new Point2D(goal);
      Point2DReadOnly start2D = new Point2D(start);

      errorMessages += assertTrue(datasetName, "Body path does not end at desired goal position: desired = " + goal + ", actual = " + pathEnd,
                                  pathEnd2D.geometricallyEquals(goal2D, START_GOAL_EPSILON));
      errorMessages += assertTrue(datasetName, "Body path does not start from desired start position: desired = " + start + ", actual = " + pathStart,
                                  pathStart2D.geometricallyEquals(start2D, START_GOAL_EPSILON));

      return errorMessages;
   }

   // TODO See if possible to support concave hulls instead of convex. It changes the shape of the regions quite some sometimes.
   private Pair<PlanarRegionsList, List<Point3D>> createVisibleRegions(PlanarRegionsList regions, Point3D observer, PlanarRegionsList knownRegions)
   {
      List<Point3D> rayImpactLocations = new ArrayList<>();
      Point3D[] pointsOnSphere = SpiralBasedAlgorithm.generatePointsOnSphere(observer, 1.0, rays);

      List<ConvexPolygon2D> visiblePolygons = new ArrayList<>();
      for (int i = 0; i < regions.getNumberOfPlanarRegions(); i++)
      {
         visiblePolygons.add(new ConvexPolygon2D());
      }

      RigidBodyTransform transform = new RigidBodyTransform();
      for (int rayIndex = 0; rayIndex < rays; rayIndex++)
      {
         Point3D pointOnSphere = pointsOnSphere[rayIndex];
         Vector3D rayDirection = new Vector3D();
         rayDirection.sub(pointOnSphere, observer);
         Point3D intersection = PlanarRegionTools.intersectRegionsWithRay(regions, observer, rayDirection);
         if (intersection == null || intersection.distanceSquared(observer) > rayLengthSquared)
         {
            continue;
         }

         rayImpactLocations.add(intersection);

         for (int regionIdx = 0; regionIdx < regions.getNumberOfPlanarRegions(); regionIdx++)
         {
            PlanarRegion region = regions.getPlanarRegion(regionIdx);
            if (PlanarRegionTools.isPointOnRegion(region, intersection, 0.01))
            {
               region.getTransformToWorld(transform);
               Point3D pointOnPlane = new Point3D(intersection);
               pointOnPlane.applyInverseTransform(transform);

               Point2D newVertex = new Point2D();
               newVertex.set(pointOnPlane);

               visiblePolygons.get(regionIdx).addVertex(newVertex);
            }
         }
      }

      PlanarRegionsList visible = new PlanarRegionsList();
      for (int i = 0; i < visiblePolygons.size(); i++)
      {
         ConvexPolygon2D polygon = visiblePolygons.get(i);
         polygon.update();
         if (polygon.getNumberOfVertices() < 2)
         {
            continue;
         }

         PlanarRegion originalRegion = regions.getPlanarRegion(i);
         originalRegion.getTransformToWorld(transform);
         PlanarRegion newRegion = new PlanarRegion(transform, polygon);
         newRegion.setRegionId(regions.getPlanarRegion(i).getRegionId());
         visible.addPlanarRegion(newRegion);
      }

      return new Pair<>(combine(knownRegions, visible), rayImpactLocations);
   }

   private PlanarRegionsList combine(PlanarRegionsList regionsA, PlanarRegionsList regionsB)
   {
      PlanarRegionsList ret = new PlanarRegionsList();

      boolean[] added = new boolean[regionsB.getNumberOfPlanarRegions()];
      for (int regionBIdx = 0; regionBIdx < regionsB.getNumberOfPlanarRegions(); regionBIdx++)
      {
         added[regionBIdx] = false;
      }

      for (PlanarRegion regionA : regionsA.getPlanarRegionsAsList())
      {
         RigidBodyTransform transformA = new RigidBodyTransform();
         regionA.getTransformToWorld(transformA);
         boolean foundMatchingRegion = false;

         for (int regionBIdx = 0; regionBIdx < regionsB.getNumberOfPlanarRegions(); regionBIdx++)
         {
            PlanarRegion regionB = regionsB.getPlanarRegion(regionBIdx);
            RigidBodyTransform transformB = new RigidBodyTransform();
            regionB.getTransformToWorld(transformB);
            if (transformA.epsilonEquals(transformB, 0.01))
            {
               ConvexPolygon2D newHull = new ConvexPolygon2D(regionA.getConvexHull(), regionB.getConvexHull());
               PlanarRegion combinedRegion = new PlanarRegion(transformA, newHull);
               combinedRegion.setRegionId(regionA.getRegionId());
               ret.addPlanarRegion(combinedRegion);
               foundMatchingRegion = true;
               added[regionBIdx] = true;
            }
         }

         if (!foundMatchingRegion)
         {
            PlanarRegion region = new PlanarRegion(transformA, new ConvexPolygon2D(regionA.getConvexHull()));
            region.setRegionId(regionA.getRegionId());
            ret.addPlanarRegion(region);
         }
      }

      for (int regionBIdx = 0; regionBIdx < regionsB.getNumberOfPlanarRegions(); regionBIdx++)
      {
         if (!added[regionBIdx])
         {
            ret.addPlanarRegion(regionsB.getPlanarRegion(regionBIdx));
         }
      }

      return ret;
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
      String dataSetName = "20190204_155900_CampLejeuneRock4";
      test.setup();
      test.runAssertionsOnDataset(dataset -> test.runAssertionsWithoutOcclusion(dataset), dataSetName);
      test.tearDown();

   }
}
