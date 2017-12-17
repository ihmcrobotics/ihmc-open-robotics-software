package us.ihmc.pathPlanning.visibilityGraphs;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import javafx.application.Application;
import javafx.stage.Stage;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityGraphsIOTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityGraphsIOTools.VisibilityGraphsUnitTestDataset;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.SimpleUIMessager;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;
import us.ihmc.pathPlanning.visibilityGraphs.visualizer.VisibilityGraphsTestVisualizer;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullDecomposition;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = IntegrationCategory.IN_DEVELOPMENT)
public class VisibilityGraphsFrameworkTest extends Application
{
   private static final long TIMEOUT = Long.MAX_VALUE; // 30000;
   private static final double START_GOAL_EPSILON = 1.0e-2;
   private static boolean VISUALIZE = true;
   private static boolean DEBUG = true;

   private static final SimpleUIMessager messager = new SimpleUIMessager(UIVisibilityGraphsTopics.API);
   private static VisibilityGraphsTestVisualizer ui;

   private static final boolean showBodyPath = true;
   private static final boolean showClusterRawPoints = false;
   private static final boolean showClusterNavigableExtrusions = false;
   private static final boolean showClusterNonNavigableExtrusions = false;
   private static final boolean showRegionInnerConnections = false;
   private static final boolean showRegionInterConnections = false;

   private static final double walkerOffsetHeight = 0.7;
   private static final double walkerRadius = 0.5;
   private static final double walkerMarchingSpeed = 0.05;

   @Before
   public void setup() throws InterruptedException, IOException
   {
      VISUALIZE = VISUALIZE && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
      DEBUG = (VISUALIZE || (DEBUG && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer()));

      if (VISUALIZE)
      {
         messager.startMessager();

         new Thread(() -> launch()).start();

         while (ui == null)
            ThreadTools.sleep(200);

         messager.submitMessage(UIVisibilityGraphsTopics.ShowBodyPath, showBodyPath);
         messager.submitMessage(UIVisibilityGraphsTopics.ShowClusterRawPoints, showClusterRawPoints);
         messager.submitMessage(UIVisibilityGraphsTopics.ShowClusterNavigableExtrusions, showClusterNavigableExtrusions);
         messager.submitMessage(UIVisibilityGraphsTopics.ShowClusterNonNavigableExtrusions, showClusterNonNavigableExtrusions);
         messager.submitMessage(UIVisibilityGraphsTopics.ShowLocalGraphs, showRegionInnerConnections);
         messager.submitMessage(UIVisibilityGraphsTopics.ShowInterConnections, showRegionInterConnections);
         messager.submitMessage(UIVisibilityGraphsTopics.WalkerOffsetHeight, walkerOffsetHeight);
         messager.submitMessage(UIVisibilityGraphsTopics.WalkerSize, walkerRadius);
      }
   }

   @After
   public void tearDown() throws Exception
   {
      if (VISUALIZE)
         stop();
   }

   @Test(timeout = TIMEOUT)
   public void testASolutionExists() throws Exception
   {
      List<VisibilityGraphsUnitTestDataset> allDatasets = VisibilityGraphsIOTools.loadAllDatasets(getClass());

      if (DEBUG)
      {
         PrintTools.info("Unit test files found: " + allDatasets.size());
      }

      AtomicReference<Boolean> previousDatasetRequested = null;
      AtomicReference<Boolean> nextDatasetRequested = null;
      AtomicReference<String> requestedDatasetPathReference = null;

      if (VISUALIZE)
      {
         List<String> allDatasetNames = allDatasets.stream().map(VisibilityGraphsUnitTestDataset::getDatasetName).collect(Collectors.toList());
         messager.submitMessage(UIVisibilityGraphsTopics.AllDatasetPaths, allDatasetNames);

         nextDatasetRequested = messager.createInput(UIVisibilityGraphsTopics.NextDatasetRequest, false);
         previousDatasetRequested = messager.createInput(UIVisibilityGraphsTopics.PreviousDatasetRequest, false);
         requestedDatasetPathReference = messager.createInput(UIVisibilityGraphsTopics.CurrentDatasetPath, null);
      }

      int numberOfFailingDatasets = 0;
      String errorMessages = "";

      int currentDatasetIndex = 0;
      if (allDatasets.isEmpty())
         Assert.fail("Did not find any datasets to test.");

      VisibilityGraphsUnitTestDataset dataset = allDatasets.get(currentDatasetIndex);

      while (dataset != null)
      {
         if (VISUALIZE)
         {
            messager.submitMessage(UIVisibilityGraphsTopics.GlobalReset, true);
            messager.submitMessage(UIVisibilityGraphsTopics.CurrentDatasetPath, dataset.getDatasetName());
         }

         String errorMessagesForCurrentFile = testFile(dataset);
         if (!errorMessagesForCurrentFile.isEmpty())
            numberOfFailingDatasets++;
         errorMessages += errorMessagesForCurrentFile;

         if (VISUALIZE)
         {
            messager.submitMessage(UIVisibilityGraphsTopics.NextDatasetRequest, false);
            messager.submitMessage(UIVisibilityGraphsTopics.PreviousDatasetRequest, false);

            while (!nextDatasetRequested.get() && !previousDatasetRequested.get() && dataset.getDatasetName().equals(requestedDatasetPathReference.get()))
            {
               if (!messager.isMessagerOpen())
                  return; // The ui has been closed

               ThreadTools.sleep(200);
            }

            if (nextDatasetRequested.get() && currentDatasetIndex < allDatasets.size() - 1)
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

   private String testFile(VisibilityGraphsUnitTestDataset dataset)
   {
      if (DEBUG)
      {
         PrintTools.info("Processing file: " + dataset.getDatasetName());
      }

      NavigableRegionsManager manager = new NavigableRegionsManager();
      PlanarRegionsList planarRegionsList = dataset.getPlanarRegionsList();

      if (VISUALIZE)
      {
         messager.submitMessage(UIVisibilityGraphsTopics.PlanarRegionData, planarRegionsList);
         messager.submitMessage(UIVisibilityGraphsTopics.StartPosition, dataset.getStart());
         messager.submitMessage(UIVisibilityGraphsTopics.GoalPosition, dataset.getGoal());
      }

      manager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = null;

      try
      {
         path = manager.calculateBodyPath(dataset.getStart(), dataset.getGoal());
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
         messager.submitMessage(UIVisibilityGraphsTopics.NavigableRegionData, manager.getListOfLocalPlanners());
         messager.submitMessage(UIVisibilityGraphsTopics.InterRegionConnectionData, manager.getConnectionPoints());
      }

      String errorMessages = "";

      errorMessages += assertTrue(dataset, "Path is null!", path != null);
      if (!errorMessages.isEmpty())
         return addPrefixToErrorMessages(dataset, errorMessages); // Cannot test anything else when no path is returned.

      errorMessages += assertTrue(dataset, "Path does not contain any waypoints", path.size() > 0);

      if (!errorMessages.isEmpty())
         return addPrefixToErrorMessages(dataset, errorMessages); // Cannot test anything else when no path is returned.

      if (dataset.hasExpectedPathSize())
         errorMessages += assertTrue(dataset, "Path size is not equal: expected = " + dataset.getExpectedPathSize() + ", actual = " + path.size(),
                                     path.size() == dataset.getExpectedPathSize());

      Point3DReadOnly pathEnd = path.get(path.size() - 1);
      Point3DReadOnly pathStart = path.get(0);
      errorMessages += assertTrue(dataset, "Body path does not end at desired goal position: desired = " + dataset.getGoal() + ", actual = " + pathEnd,
                                  pathEnd.geometricallyEquals(dataset.getGoal(), START_GOAL_EPSILON));
      errorMessages += assertTrue(dataset, "Body path does not start from desired start position: desired = " + dataset.getStart() + ", actual = " + pathStart,
                                  pathStart.geometricallyEquals(dataset.getStart(), START_GOAL_EPSILON));

      // "Walk" along the body path and assert that the walker does not go through any region.
      int currentSegmentIndex = 0;
      Point3D walkerCurrentPosition = new Point3D(pathStart);
      List<Point3D> collisions = new ArrayList<>();

      while (!walkerCurrentPosition.geometricallyEquals(pathEnd, 1.0e-3))
      {
         for (PlanarRegion planarRegion : planarRegionsList.getPlanarRegionsAsList())
         {
            Point3D walkerBody3D = new Point3D(walkerCurrentPosition);
            walkerBody3D.addZ(walkerOffsetHeight);

            Plane3D plane = planarRegion.getPlane();
            double distance = plane.distance(walkerBody3D);
            if (distance > walkerRadius)
               continue;

            RigidBodyTransform transformToWorld = new RigidBodyTransform();
            planarRegion.getTransformToWorld(transformToWorld);
            walkerBody3D.applyInverseTransform(transformToWorld);
            Point2D walkerBody2D = new Point2D(walkerBody3D);

            if (planarRegion.getNumberOfConvexPolygons() == 0)
            {
               List<Point2DReadOnly> concaveHullVertices = new ArrayList<>(Arrays.asList(planarRegion.getConcaveHull()));
               double depthThreshold = 0.05;
               List<ConvexPolygon2D> convexPolygons = new ArrayList<>();
               ConcaveHullDecomposition.recursiveApproximateDecomposition(concaveHullVertices, depthThreshold, convexPolygons);
            }

            for (int i = 0; i < planarRegion.getNumberOfConvexPolygons(); i++)
            {
               double distanceXY = planarRegion.getConvexPolygon(i).distance(walkerBody2D);
               distance = Math.sqrt(EuclidCoreTools.normSquared(distanceXY, walkerBody3D.getZ()));

               if (distance < walkerRadius)
               {
                  Point2D intersectionLocal = planarRegion.getConvexPolygon(i).orthogonalProjectionCopy(walkerBody2D);
                  if (intersectionLocal == null) // walkerBody2D is inside the polygon
                     intersectionLocal = walkerBody2D;
                  Point3D intersectionWorld = toWorld(intersectionLocal, transformToWorld);
                  errorMessages += fail(dataset, "Body path is going through a region at: " + intersectionWorld + ", distance from region: " + distance);
                  collisions.add(intersectionWorld);
               }
            }
         }

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
         messager.submitMessage(UIVisibilityGraphsTopics.WalkerCollisionLocations, collisions);

      return addPrefixToErrorMessages(dataset, errorMessages);
   }

   private static Point3D toWorld(Point2DReadOnly pointLocal, Transform transformToWorld)
   {
      Point3D inWorld = new Point3D(pointLocal);
      transformToWorld.transform(inWorld);
      return inWorld;
   }

   private static String addPrefixToErrorMessages(VisibilityGraphsUnitTestDataset dataset, String errorMessages)
   {

      if (!errorMessages.isEmpty())
         return "\n" + dataset.getDatasetName() + errorMessages;
      else
         return "";
   }

   private String fail(VisibilityGraphsUnitTestDataset dataset, String message)
   {
      return assertTrue(dataset, message, false);
   }

   private String assertTrue(VisibilityGraphsUnitTestDataset dataset, String message, boolean condition)
   {
      if (VISUALIZE || DEBUG)
      {
         if (!condition)
            PrintTools.error(dataset.getDatasetName() + ": " + message);
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
}
