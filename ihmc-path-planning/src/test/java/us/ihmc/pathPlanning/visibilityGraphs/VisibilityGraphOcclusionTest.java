package us.ihmc.pathPlanning.visibilityGraphs;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.shape.primitives.Ellipsoid3D;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.NavigableRegionFilter;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.PlanarRegionFilter;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullDecomposition;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class VisibilityGraphOcclusionTest
{
   private boolean visualize = false;
   private PlanarRegionsList occludedEnvironmentWithAGoalPlane;
   private PlanarRegionsList occludedEnvironmentWithoutAGoalPlane;
   private PlanarRegionsList smallWallWithNoGround;
   private PlanarRegionsList tinyWallWithNoGround;
   private PlanarRegionsList largeWallWithNoGround;
   private VisibilityGraphsParametersReadOnly visibilityGraphsParameters;

   // For enabling helpful prints.
   private static boolean DEBUG = true;

   private static VisibilityGraphsTestVisualizerApplication visualizerApplication = null;
   // Because we use JavaFX, there will be two instance of VisibilityGraphsFrameworkTest, one for running the test and one starting the ui. The messager has to be static so both the ui and test use the same instance.
   private static JavaFXMessager messager = null;


   // The following are used for collision checks.
   private static final double walkerOffsetHeight = 0.75;
   private static final Vector3D walkerRadii = new Vector3D(0.25, 0.25, 0.5);
   private static final double walkerMarchingSpeed = 0.2;

   @BeforeEach
   public void setup()
   {
      DEBUG = (visualize || (DEBUG && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer()));

      visualize = visualize && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
      occludedEnvironmentWithAGoalPlane = simpleOccludedEnvironment(true);
      occludedEnvironmentWithoutAGoalPlane = simpleOccludedEnvironment(false);
      smallWallWithNoGround = smallWallWithNoGoalGroundEnvironment();
      tinyWallWithNoGround = tinyWallWithNoGoalGroundEnvironment();
      largeWallWithNoGround = largeWallWithNoGoalGroundEnvironment();
      visibilityGraphsParameters = new DefaultVisibilityGraphParameters();

      if (visualize)
      {
         visualizerApplication = new VisibilityGraphsTestVisualizerApplication();
         visualizerApplication.startOnAThread();

         messager = visualizerApplication.getMessager();
      }
   }

   @AfterEach
   public void tearDown() throws Exception
   {
      occludedEnvironmentWithAGoalPlane = null;
      occludedEnvironmentWithoutAGoalPlane = null;
      smallWallWithNoGround = null;
      tinyWallWithNoGround = null;
      largeWallWithNoGround = null;
      visibilityGraphsParameters = null;

      if (visualize)
      {
         visualizerApplication.stop();
         visualizerApplication = null;
         messager = null;
      }
   }


   @Test
   public void testVisibilityGraphWithOcclusion()
   {
      Point3D goal = new Point3D(2.0, -1.0, 0.0);

      runTest(occludedEnvironmentWithAGoalPlane, goal);
   }

   @Test
   public void testVisibilityGraphWithOcclusionAndNoGoalPlane()
   {
      Point3D goal = new Point3D(2.0, -1.0, 0.0);

      runTest(occludedEnvironmentWithoutAGoalPlane, goal);
   }

   @Test
   public void testSmallWallWithNoGoalPlane()
   {
      Point3D goal = new Point3D(3.0, 0.0, 0.0);

      runTest(smallWallWithNoGround, goal);
   }

   @Test
   public void testTinyWallWithNoGoalPlane()
   {
      Point3D goal = new Point3D(3.0, 0.0, 0.0);

      runTest(tinyWallWithNoGround, goal);
   }

   @Test
   public void testLargeWallWithNoGoalPlane()
   {
      Point3D goal = new Point3D(3.0, 0.0, 0.0);

      runTest(largeWallWithNoGround, goal);
   }


   private void runTest(PlanarRegionsList planarRegionsList, Point3D goal)
   {
      Point3D start = new Point3D();

      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(planarRegionsList.getPlanarRegionsAsList());
      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(start, goal);

      if(visualize)
      {
         visualizerApplication.submitPlanarRegionsListToVisualizer(planarRegionsList);
         visualizerApplication.submitGoalToVisualizer(goal);
         visualizerApplication.submitStartToVisualizer(start);
         visualizerApplication.submitNavigableRegionsToVisualizer(navigableRegionsManager.getNavigableRegionsList());
         messager.submitMessage(UIVisibilityGraphsTopics.BodyPathData, path);
         visualizerApplication.submitVisibilityGraphSolutionToVisualizer(navigableRegionsManager.getVisibilityMapSolution());
      }

      String errorMessages = testBodyPath(planarRegionsList, path);

      if (visualize)
         ThreadTools.sleepForever();

      Assert.assertTrue("Errors: " + errorMessages, errorMessages.isEmpty());
      LogTools.info("Finished testing.");


   }

   private String testBodyPath(PlanarRegionsList planarRegionsList, List<Point3DReadOnly> path)
   {
      // "Walk" along the body path and assert that the walker does not go through any region.
      int currentSegmentIndex = 0;
      Point3DReadOnly pathStart = path.get(0);
      Point3DReadOnly pathEnd = path.get(path.size() - 1);
      Point3D walkerCurrentPosition = new Point3D(pathStart);
      List<Point3D> collisions = new ArrayList<>();
      Ellipsoid3D walkerShape = new Ellipsoid3D();
      walkerShape.setRadii(walkerRadii);

      Point3D walkerBody3D = new Point3D(walkerCurrentPosition);
      walkerBody3D.addZ(walkerOffsetHeight);
      walkerShape.getPosition().set(walkerBody3D);

      String errorMessages = walkerCollisionChecks(walkerShape, planarRegionsList, collisions);

      while (!walkerCurrentPosition.geometricallyEquals(pathEnd, 1.0e-2))
      {
         walkerBody3D = new Point3D(walkerCurrentPosition);
         walkerBody3D.addZ(walkerOffsetHeight);
         walkerShape.getPosition().set(walkerBody3D);

         errorMessages += walkerCollisionChecks(walkerShape, planarRegionsList, collisions);

         //         walkerCurrentPosition.set(travelAlongBodyPath(walkerMarchingSpeed, walkerCurrentPosition, path));
         Point3DReadOnly segmentStart = path.get(currentSegmentIndex);
         Point3DReadOnly segmentEnd = path.get(currentSegmentIndex + 1);


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

      if (visualize)
         messager.submitMessage(UIVisibilityGraphsTopics.WalkerCollisionLocations, collisions);

      return errorMessages;
   }

   private String walkerCollisionChecks(Ellipsoid3DReadOnly walkerShapeWorld, PlanarRegionsList planarRegionsList, List<Point3D> collisionsToPack)
   {
      String errorMessages = "";
      NavigableRegionFilter navigableFilter = visibilityGraphsParameters.getNavigableRegionFilter();
      PlanarRegionFilter planarRegionFilter = visibilityGraphsParameters.getPlanarRegionFilter();
      List<PlanarRegion> regionsToCheck = planarRegionsList.getPlanarRegionsAsList().stream().filter(query -> navigableFilter.isPlanarRegionNavigable(query, planarRegionsList.getPlanarRegionsAsList())).collect(
            Collectors.toList());
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
                  errorMessages += fail("Body path is going through a region.");
                  break;
               }
               else
               {
                  throw new RuntimeException("Not sure what went wrong to here.");
               }
            }
            closestPoint = new Point3D(closestPoint2D);

            if (walkerShapeLocal.isPointInside(closestPoint))
            {
               Point2DBasics intersectionLocal = closestPoint2D;
               Point3D intersectionWorld = new Point3D(intersectionLocal);
               planarRegion.transformFromLocalToWorld(intersectionWorld);
               errorMessages += fail("Walker is going through a region at: " + intersectionWorld);
               collisionsToPack.add(intersectionWorld);
            }
         }
      }

      return errorMessages;
   }

   private String fail(String message)
   {
      return assertTrue(message, false);
   }

   private String assertTrue(String message, boolean condition)
   {
      if (visualize || DEBUG)
      {
         if (!condition)
            LogTools.error(": " + message);
      }
      return !condition ? "\n" + message : "";
   }

   private static PlanarRegionsList simpleOccludedEnvironment(boolean includeGoalPlane)
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.addRectangle(2.0, 4.0);

      generator.translate(1.0, -1.0, 0.5);
      generator.rotate(0.5 * Math.PI, Axis.Y);

      generator.addRectangle(0.9, 1.9);

      if (includeGoalPlane)
      {
         generator.identity();
         generator.translate(2.0, -1.0, 0.0);
         generator.addRectangle(1.0, 1.0);
      }

      return generator.getPlanarRegionsList();
   }

   private static PlanarRegionsList smallWallWithNoGoalGroundEnvironment()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.addRectangle(2.0, 4.0);

      generator.translate(2.0, 0.0, 1.0);
      generator.rotate(0.5 * Math.PI, Axis.Y);

      generator.addRectangle(0.2, 0.5);

      generator.identity();

      return generator.getPlanarRegionsList();
   }

   private static PlanarRegionsList tinyWallWithNoGoalGroundEnvironment()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.addRectangle(2.0, 4.0);

      generator.translate(2.0, 0.0, 1.0);
      generator.rotate(0.5 * Math.PI, Axis.Y);

      generator.addRectangle(0.05, 0.05);

      generator.identity();

      return generator.getPlanarRegionsList();
   }

   private static PlanarRegionsList largeWallWithNoGoalGroundEnvironment()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.addRectangle(2.0, 4.0);

      generator.translate(2.0, 0.0, 0.5);
      generator.rotate(0.5 * Math.PI, Axis.Y);

      generator.addRectangle(1.0, 0.6);

      generator.identity();

      return generator.getPlanarRegionsList();
   }

   private class TestParameters extends DefaultVisibilityGraphParameters
   {
      @Override
      public double getSearchHostRegionEpsilon()
      {
         return 0;
      }
   }
}
