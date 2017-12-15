package us.ihmc.pathPlanning.visibilityGraphs;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

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
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.VisibilityGraphsIOTools.VisibilityGraphsUnitTestDataset;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.SimpleUIMessager;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;
import us.ihmc.pathPlanning.visibilityGraphs.visualizer.VisibilityGraphsTestVisualizer;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullCollection;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullDecomposition;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = IntegrationCategory.IN_DEVELOPMENT)
public class VisibilityGraphsFrameworkTest extends Application
{
   private static final long TIMEOUT = 30000;
   private static final double START_GOAL_EPSILON = 1.0e-2;
   private static boolean VISUALIZE = true;
   private boolean DEBUG = true;

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
      List<VisibilityGraphsUnitTestDataset> allDatasets = VisibilityGraphsIOTools.loadAllDatasets();

      if (DEBUG)
      {
         PrintTools.info("Unit test files found: " + allDatasets.size());
      }

      AtomicReference<Boolean> nextDatasetRequested = null;
      if (VISUALIZE)
         nextDatasetRequested = messager.createInput(UIVisibilityGraphsTopics.NextDatasetRequest, false);

      for (VisibilityGraphsUnitTestDataset dataset : allDatasets)
      {
         if (VISUALIZE)
            messager.submitMessage(UIVisibilityGraphsTopics.GlobalReset, true);

         testFile(dataset);

         if (VISUALIZE)
         {
            messager.submitMessage(UIVisibilityGraphsTopics.NextDatasetRequest, false);

            while (!nextDatasetRequested.get())
            {
               if (!messager.isMessagerOpen())
                  return; // The ui has been closed

               ThreadTools.sleep(200);
            }
         }
      }
   }

   private void testFile(VisibilityGraphsUnitTestDataset dataset)
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

      assertTrue("Path is null!", path != null);
      if (path == null)
         return; // Cannot test anything else when no path is returned.

      assertTrue("Path does not contain any waypoints", path.size() > 0);

      if (dataset.hasExpectedPathSize())
         assertTrue("Path size is not equal", path.size() == dataset.getExpectedPathSize());

      Point3DReadOnly pathEnd = path.get(path.size() - 1);
      Point3DReadOnly pathStart = path.get(0);
      assertTrue("Body path does not end at desired goal position: desired = " + dataset.getGoal() + ", actual = " + pathEnd,
                 pathEnd.geometricallyEquals(dataset.getGoal(), START_GOAL_EPSILON));
      assertTrue("Body path does not start from desired start position: desired = " + dataset.getGoal() + ", actual = " + pathEnd,
                 pathStart.geometricallyEquals(dataset.getStart(), START_GOAL_EPSILON));

      // "Walk" along the body path and assert that the walker does not go through any region.
      Point3D walkerCurrentPosition = new Point3D(pathStart);
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
               distance = planarRegion.getConvexPolygon(i).distance(walkerBody2D);

               if (distance < walkerRadius)
               {
                  fail("Body path is going through a region");
                  break;
               }
            }
         }

         walkerCurrentPosition = travelAlongBodyPath(walkerMarchingSpeed, walkerCurrentPosition, path);
      }
   }

   private static Point3D travelAlongBodyPath(double distanceToTravel, Point3D startingPosition, List<Point3DReadOnly> bodyPath)
   {
      Point3D newPosition = new Point3D();

      for (int i = 0; i < bodyPath.size() - 1; i++)
      {
         LineSegment3D segment = new LineSegment3D(bodyPath.get(i), bodyPath.get(i + 1));

         if (segment.distance(startingPosition) < 1.0e-4)
         {
            Vector3D segmentDirection = segment.getDirection(true);
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

   private void fail(String message)
   {
      assertTrue(message, false);
   }

   private void assertTrue(String message, boolean condition)
   {
      if (VISUALIZE)
      {
         if (!condition)
            PrintTools.error(message);
      }
      else
      {
         Assert.assertTrue(message, condition);
      }
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
