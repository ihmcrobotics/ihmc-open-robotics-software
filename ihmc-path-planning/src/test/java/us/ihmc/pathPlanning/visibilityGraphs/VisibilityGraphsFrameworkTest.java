package us.ihmc.pathPlanning.visibilityGraphs;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import javafx.application.Application;
import javafx.stage.Stage;
import javafx.util.Pair;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Ellipsoid3D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.NavigableExtrusionDistanceCalculator;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityGraphsIOTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityGraphsIOTools.VisibilityGraphsUnitTestDataset;
import us.ihmc.pathPlanning.visibilityGraphs.ui.VisibilityGraphsDataExporter;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.SimpleUIMessager;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;
import us.ihmc.pathPlanning.visibilityGraphs.visualizer.VisibilityGraphsTestVisualizer;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullDecomposition;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.SpiralBasedAlgorithm;

public class VisibilityGraphsFrameworkTest extends Application
{
   // Set that to MAX_VALUE when visualizing. Before pushing, it has to be reset to a reasonable value.
   private static final long TIMEOUT = 100000; // Long.MAX_VALUE; // 
   // Threshold used to assert that the body path starts and ends where we asked it to.
   private static final double START_GOAL_EPSILON = 1.0e-2;

   // Whether to start the UI or not.
   private static boolean VISUALIZE = false;
   // For enabling helpful prints.
   private static boolean DEBUG = true;

   // Because we use JavaFX, there will be two instance of VisibilityGraphsFrameworkTest, one for running the test and one starting the ui. The messager has to be static so both the ui and test use the same instance.
   private static SimpleUIMessager messager = null;
   // Because JavaFX will create a fresh new instance of VisibilityGraphsFrameworkTest, the ui has to be static so there is only one instance and we can refer to it in the test part.
   private static VisibilityGraphsTestVisualizer ui;

   // Default UI parameters which should be changeable on the fly
   private static final boolean showBodyPath = true;
   private static final boolean showClusterRawPoints = false;
   private static final boolean showClusterNavigableExtrusions = false;
   private static final boolean showClusterNonNavigableExtrusions = false;
   private static final boolean showRegionInnerConnections = false;
   private static final boolean showRegionInterConnections = false;

   // The following are used for collision checks.
   private static final double walkerOffsetHeight = 0.75;
   private static final Vector3D walkerRadii = new Vector3D(0.25, 0.25, 0.5);
   private static final double walkerMarchingSpeed = 0.05;

   // For the occlusion test
   private static final int rays = 5000;
   private static final double rayLengthSquared = MathTools.square(5.0);

   private static VisibilityGraphsParameters createTestParameters()
   {
      return new DefaultVisibilityGraphParameters()
      {
         @Override
         public double getTooHighToStepDistance()
         {
            return 0.4;
         }

         @Override
         public double getExtrusionDistance()
         {
            return 0.4;
         }

         @Override
         public NavigableExtrusionDistanceCalculator getNavigableExtrusionDistanceCalculator()
         {
            return new NavigableExtrusionDistanceCalculator()
            {
               @Override
               public double computeExtrusionDistance(PlanarRegion navigableRegionToBeExtruded)
               {
                  return 0.01;
               }
            };
         }
      };
   }

   @Before
   public void setup() throws InterruptedException, IOException
   {
      VISUALIZE = VISUALIZE && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
      DEBUG = (VISUALIZE || (DEBUG && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer()));

      if (VISUALIZE)
      {
         messager = new SimpleUIMessager(UIVisibilityGraphsTopics.API);
         messager.startMessager();

         // Did not find a better solution for starting JavaFX and still be able to move on.
         new Thread(() -> launch()).start();

         while (ui == null)
            ThreadTools.sleep(200);

         messager.submitMessage(UIVisibilityGraphsTopics.ShowBodyPath, showBodyPath);
         messager.submitMessage(UIVisibilityGraphsTopics.ShowClusterRawPoints, showClusterRawPoints);
         messager.submitMessage(UIVisibilityGraphsTopics.ShowClusterNavigableExtrusions, showClusterNavigableExtrusions);
         messager.submitMessage(UIVisibilityGraphsTopics.ShowClusterNonNavigableExtrusions, showClusterNonNavigableExtrusions);
         messager.submitMessage(UIVisibilityGraphsTopics.ShowNavigableRegionVisibilityMaps, showRegionInnerConnections);
         messager.submitMessage(UIVisibilityGraphsTopics.ShowInterRegionVisibilityMap, showRegionInterConnections);
      }
   }

   @After
   public void tearDown() throws Exception
   {
      if (VISUALIZE)
      {
         stop();
         ui = null;
         messager = null;
      }
   }

   @Test(timeout = TIMEOUT)
   @ContinuousIntegrationTest(estimatedDuration = 13.0, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   public void testDatasetsWithoutOcclusion() throws Exception
   {
      if (VISUALIZE)
      {
         messager.submitMessage(UIVisibilityGraphsTopics.EnableWalkerAnimation, true);
         messager.submitMessage(UIVisibilityGraphsTopics.WalkerOffsetHeight, walkerOffsetHeight);
         messager.submitMessage(UIVisibilityGraphsTopics.WalkerSize, walkerRadii);
      }
      runAssertionsOnAllDatasets(dataset -> runAssertionsWithoutOcclusion(dataset));
   }

   @Test(timeout = TIMEOUT)
   @ContinuousIntegrationTest(estimatedDuration = 10.0, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   public void testDatasetsNoOcclusionSimulateDynamicReplanning() throws Exception
   {
      if (VISUALIZE)
      {
         messager.submitMessage(UIVisibilityGraphsTopics.EnableWalkerAnimation, false);
      }
      runAssertionsOnAllDatasets(dataset -> runAssertionsSimulateDynamicReplanning(dataset, 0.20, 1000, false));
   }

   @Test(timeout = TIMEOUT)
   @ContinuousIntegrationTest(estimatedDuration = 10.0, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   public void testDatasetsSimulateOcclusionAndDynamicReplanning() throws Exception
   {
      if (VISUALIZE)
      {
         messager.submitMessage(UIVisibilityGraphsTopics.EnableWalkerAnimation, false);
      }
      runAssertionsOnAllDatasets(dataset -> runAssertionsSimulateDynamicReplanning(dataset, 0.20, 1000, true));
   }

   private void runAssertionsOnAllDatasets(DatasetTestRunner datasetTestRunner) throws Exception
   {
      List<VisibilityGraphsUnitTestDataset> allDatasets = VisibilityGraphsIOTools.loadAllDatasets(VisibilityGraphsDataExporter.class);

      if (DEBUG)
      {
         PrintTools.info("Unit test files found: " + allDatasets.size());
      }

      AtomicReference<Boolean> previousDatasetRequested = null;
      AtomicReference<Boolean> reloadDatasetRequested = null;
      AtomicReference<Boolean> nextDatasetRequested = null;
      AtomicReference<String> requestedDatasetPathReference = null;

      if (VISUALIZE)
      {
         List<String> allDatasetNames = allDatasets.stream().map(VisibilityGraphsUnitTestDataset::getDatasetName).collect(Collectors.toList());
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
      allDatasets.stream().map(VisibilityGraphsUnitTestDataset::getPlanarRegionsList).map(PlanarRegionsList::getPlanarRegionsAsList)
                 .forEach(regionsList -> regionsList.forEach(region -> region.setRegionId(random.nextInt())));

      VisibilityGraphsUnitTestDataset dataset = allDatasets.get(currentDatasetIndex);

      while (dataset != null)
      {
         if (VISUALIZE)
         {
            messager.submitMessage(UIVisibilityGraphsTopics.GlobalReset, true);
            messager.submitMessage(UIVisibilityGraphsTopics.CurrentDatasetPath, dataset.getDatasetName());
         }

         if (DEBUG)
         {
            PrintTools.info("Processing file: " + dataset.getDatasetName());
         }

         String errorMessagesForCurrentFile = datasetTestRunner.testDataset(dataset);
         if (!errorMessagesForCurrentFile.isEmpty())
            numberOfFailingDatasets++;
         errorMessages += errorMessagesForCurrentFile;

         if (VISUALIZE)
         {
            messager.submitMessage(UIVisibilityGraphsTopics.NextDatasetRequest, false);
            messager.submitMessage(UIVisibilityGraphsTopics.ReloadDatasetRequest, false);
            messager.submitMessage(UIVisibilityGraphsTopics.PreviousDatasetRequest, false);

            while (!nextDatasetRequested.get() && !reloadDatasetRequested.get() && !previousDatasetRequested.get()
                  && dataset.getDatasetName().equals(requestedDatasetPathReference.get()))
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
               VisibilityGraphsUnitTestDataset requestedDataset = allDatasets.stream().filter(d -> d.getDatasetName().equals(path)).findFirst().orElse(null);
               if (requestedDataset == null)
               {
                  PrintTools.error("Could not find the requested dataset with name: " + path);
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

   private String runAssertionsWithoutOcclusion(VisibilityGraphsUnitTestDataset dataset)
   {
      String datasetName = dataset.getDatasetName();

      PlanarRegionsList planarRegionsList = dataset.getPlanarRegionsList();

      Point3D start = dataset.getStart();
      Point3D goal = dataset.getGoal();

      if (VISUALIZE)
      {
         messager.submitMessage(UIVisibilityGraphsTopics.PlanarRegionData, planarRegionsList);
         messager.submitMessage(UIVisibilityGraphsTopics.StartPosition, start);
         messager.submitMessage(UIVisibilityGraphsTopics.GoalPosition, goal);
      }

      String errorMessages = calculateAndTestVizGraphsBodyPath(datasetName, start, goal, planarRegionsList);

      return addPrefixToErrorMessages(datasetName, errorMessages);
   }

   private String runAssertionsSimulateDynamicReplanning(VisibilityGraphsUnitTestDataset dataset, double walkerSpeed, long maxSolveTimeInMilliseconds,
                                                         boolean simulateOcclusions)
   {
      String datasetName = dataset.getDatasetName();

      PlanarRegionsList planarRegionsList = dataset.getPlanarRegionsList();

      Point3D start = dataset.getStart();
      Point3D goal = dataset.getGoal();
      AtomicReference<Boolean> stopWalkerRequest = null;

      if (VISUALIZE)
      {
         stopWalkerRequest = messager.createInput(UIVisibilityGraphsTopics.StopWalker, false);
         if (simulateOcclusions)
            messager.submitMessage(UIVisibilityGraphsTopics.ShadowPlanarRegionData, planarRegionsList);
         else
            messager.submitMessage(UIVisibilityGraphsTopics.PlanarRegionData, planarRegionsList);
         messager.submitMessage(UIVisibilityGraphsTopics.StartPosition, start);
         messager.submitMessage(UIVisibilityGraphsTopics.GoalPosition, goal);
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
               messager.submitMessage(UIVisibilityGraphsTopics.PlanarRegionData, knownRegions);
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

   private static Point3D travelAlongBodyPath(double distanceToTravel, Point3D startingPosition, List<Point3DReadOnly> bodyPath)
   {
      Point3D newPosition = new Point3D();

      for (int i = 0; i < bodyPath.size() - 1; i++)
      {
         LineSegment3D segment = new LineSegment3D(bodyPath.get(i), bodyPath.get(i + 1));

         if (segment.distance(startingPosition) < 1.0e-4)
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

   private String calculateAndTestVizGraphsBodyPath(String datasetName, Point3D start, Point3D goal, PlanarRegionsList planarRegionsList)
   {
      return calculateAndTestVizGraphsBodyPath(datasetName, start, goal, planarRegionsList, null);
   }

   private String calculateAndTestVizGraphsBodyPath(String datasetName, Point3D start, Point3D goal, PlanarRegionsList planarRegionsList,
                                                    List<Point3DReadOnly> bodyPathToPack)
   {
      VisibilityGraphsParameters parameters = createTestParameters();
      NavigableRegionsManager manager = new NavigableRegionsManager(parameters);
      manager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = null;

      try
      {
         path = manager.calculateBodyPath(start, goal);
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
         messager.submitMessage(UIVisibilityGraphsTopics.NavigableRegionData, manager.getNavigableRegions());
         messager.submitMessage(UIVisibilityGraphsTopics.StartVisibilityMap, manager.getStartMap());
         messager.submitMessage(UIVisibilityGraphsTopics.GoalVisibilityMap, manager.getGoalMap());
         messager.submitMessage(UIVisibilityGraphsTopics.NavigableRegionVisibilityMap, manager.getNavigableRegions());
         messager.submitMessage(UIVisibilityGraphsTopics.InterRegionVisibilityMap, manager.getInterRegionConnections());
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
         walkerShape.setPosition(walkerBody3D);

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

         if (!walkerShapeWorld.isInsideOrOnSurface(closestPoint))
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
            Point2D closestPoint2D = convexPolygon.orthogonalProjectionCopy(walkerPosition2D);
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

            if (walkerShapeLocal.isInsideOrOnSurface(closestPoint))
            {
               Point2D intersectionLocal = closestPoint2D;
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
      errorMessages += assertTrue(datasetName, "Body path does not end at desired goal position: desired = " + goal + ", actual = " + pathEnd,
                                  pathEnd.geometricallyEquals(goal, START_GOAL_EPSILON));
      errorMessages += assertTrue(datasetName, "Body path does not start from desired start position: desired = " + start + ", actual = " + pathStart,
                                  pathStart.geometricallyEquals(start, START_GOAL_EPSILON));

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
            PrintTools.error(datasetName + ": " + message);
      }
      return !condition ? "\n" + message : "";
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      ui = new VisibilityGraphsTestVisualizer(primaryStage, messager);
      ui.show();
   }

   @Override
   public void stop() throws Exception
   {
      ui.stop();
   }

   private static interface DatasetTestRunner
   {
      String testDataset(VisibilityGraphsUnitTestDataset dataset);
   }
}
