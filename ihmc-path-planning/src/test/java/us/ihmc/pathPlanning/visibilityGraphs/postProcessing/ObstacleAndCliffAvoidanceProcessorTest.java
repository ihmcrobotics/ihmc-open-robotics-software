package us.ihmc.pathPlanning.visibilityGraphs.postProcessing;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegionsFactory;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.NavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pathPlanning.visibilityGraphs.tools.ClusterTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.TestEnvironmentTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static us.ihmc.robotics.Assert.*;

public class ObstacleAndCliffAvoidanceProcessorTest
{
   private static final long timeout = 30000;
   private static final double epsilon = 1e-10;
   private static final int iters = 500;

   private static final boolean visualize = false;

   @Test
   public void testRemoveDuplicated3DPointsFromList()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double distanceToFilter = RandomNumbers.nextDouble(random, 1e-4, 1.0);
         int numberOfPointsRemove = RandomNumbers.nextInt(random, 0, 10);
         int numberOfPointsToCreate = RandomNumbers.nextInt(random, numberOfPointsRemove, 50);

         List<Point3D> listOfPoints = new ArrayList<>();
         Point3D startPoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         listOfPoints.add(startPoint);
         int numberOfRemovablePointsMade = 0;
         for (int i = 1; i < numberOfPointsToCreate; i++)
         {
            int numberOfPointsRemainingToMake = numberOfPointsToCreate - listOfPoints.size();
            int numberOfRemovablePointsRemainingToMake = numberOfPointsRemove - numberOfRemovablePointsMade;

            double probabilityPointIsRemovable = (double) numberOfRemovablePointsRemainingToMake / numberOfPointsRemainingToMake;
            boolean nextPointIsRemovable = RandomNumbers.nextBoolean(random, probabilityPointIsRemovable);

            Vector3D displacementVector = EuclidCoreRandomTools.nextVector3D(random, 0.1, 1.0);
            displacementVector.normalize();

            if (nextPointIsRemovable)
            {
               displacementVector.scale(0.75 * distanceToFilter);
               numberOfRemovablePointsMade++;
            }
            else
            {
               displacementVector.scale(1.5 * distanceToFilter);
            }

            Point3D newPoint = new Point3D(listOfPoints.get(i - 1));
            newPoint.add(displacementVector);

            listOfPoints.add(newPoint);
         }

         ObstacleAndCliffAvoidanceProcessor.removeDuplicated3DPointsFromList(listOfPoints, distanceToFilter);

         for (int i = 0; i < listOfPoints.size(); i++)
         {
            for (int j = i + 1; j < listOfPoints.size(); j++)
            {
               Point3DReadOnly pointA = listOfPoints.get(i);
               Point3DReadOnly pointB = listOfPoints.get(j);

               double distance = pointA.distance(pointB);
               assertTrue("Point " + i + " = " + pointA + " is too close to point " + j + " = " + pointB + ", with a distance of " + distance
                                + ", which should be at least " + distanceToFilter, distance > distanceToFilter);
            }
         }
      }
   }

   @Test
   public void testRemoveDuplicated2DPointsFromList()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double distanceToFilter = RandomNumbers.nextDouble(random, 1e-4, 1.0);
         int numberOfPointsRemove = RandomNumbers.nextInt(random, 0, 10);
         int numberOfPointsToCreate = RandomNumbers.nextInt(random, numberOfPointsRemove, 50);

         List<Point2D> listOfPoints = new ArrayList<>();
         Point2D startPoint = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         listOfPoints.add(startPoint);
         int numberOfRemovablePointsMade = 0;
         for (int i = 1; i < numberOfPointsToCreate; i++)
         {
            int numberOfPointsRemainingToMake = numberOfPointsToCreate - listOfPoints.size();
            int numberOfRemovablePointsRemainingToMake = numberOfPointsRemove - numberOfRemovablePointsMade;

            double probabilityPointIsRemovable = (double) numberOfRemovablePointsRemainingToMake / numberOfPointsRemainingToMake;
            boolean nextPointIsRemovable = RandomNumbers.nextBoolean(random, probabilityPointIsRemovable);

            Vector2D displacementVector = EuclidCoreRandomTools.nextVector2D(random, 0.1, 1.0);
            displacementVector.normalize();

            if (nextPointIsRemovable)
            {
               displacementVector.scale(0.75 * distanceToFilter);
               numberOfRemovablePointsMade++;
            }
            else
            {
               displacementVector.scale(1.5 * distanceToFilter);
            }

            Point2D newPoint = new Point2D(listOfPoints.get(i - 1));
            newPoint.add(displacementVector);

            listOfPoints.add(newPoint);
         }

         ObstacleAndCliffAvoidanceProcessor.removeDuplicated2DPointsFromList(listOfPoints, distanceToFilter);

         for (int i = 0; i < listOfPoints.size(); i++)
         {
            for (int j = i + 1; j < listOfPoints.size(); j++)
            {
               Point2DReadOnly pointA = listOfPoints.get(i);
               Point2DReadOnly pointB = listOfPoints.get(j);

               double distance = pointA.distance(pointB);
               assertTrue("Point " + i + " = " + pointA + " is too close to point " + j + " = " + pointB + ", with a distance of " + distance
                                + ", which should be at least " + distanceToFilter, distance > distanceToFilter);
            }
         }
      }
   }

   @Test
   public void testRemoveDuplicateStartOrEndPointsFromList()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double distanceToFilter = RandomNumbers.nextDouble(random, 1e-4, 1.0);
         int numberOfPointsRemove = RandomNumbers.nextInt(random, 0, 10);
         int numberOfPointsToCreate = RandomNumbers.nextInt(random, numberOfPointsRemove, 50);

         List<Point3D> listOfPoints = new ArrayList<>();
         Point3D startPoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3D endPoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         int numberOfRemovablePointsMade = 0;
         for (int i = 0; i < numberOfPointsToCreate; i++)
         {
            int numberOfPointsRemainingToMake = numberOfPointsToCreate - listOfPoints.size();
            int numberOfRemovablePointsRemainingToMake = numberOfPointsRemove - numberOfRemovablePointsMade;

            double probabilityPointIsRemovable = (double) numberOfRemovablePointsRemainingToMake / numberOfPointsRemainingToMake;
            boolean nextPointIsRemovable = RandomNumbers.nextBoolean(random, probabilityPointIsRemovable);

            Vector3D displacementVector = EuclidCoreRandomTools.nextVector3D(random, -5.0, 5.0);
            displacementVector.normalize();

            if (nextPointIsRemovable)
            {
               displacementVector.scale(0.75 * distanceToFilter);
               numberOfRemovablePointsMade++;
            }
            else
            {
               displacementVector.scale(1.5 * distanceToFilter);
            }

            boolean displacementFromStartPoint = RandomNumbers.nextBoolean(random, 0.5);
            Point3D newPoint = new Point3D();
            if (displacementFromStartPoint)
               newPoint.set(startPoint);
            else
               newPoint.set(endPoint);
            newPoint.add(displacementVector);

            listOfPoints.add(newPoint);
         }

         ObstacleAndCliffAvoidanceProcessor.removeDuplicateStartOrEndPointsFromList(listOfPoints, startPoint, endPoint, distanceToFilter);

         for (int i = 0; i < listOfPoints.size(); i++)
         {
            Point3DReadOnly point = listOfPoints.get(i);

            double distanceFromStart = point.distance(startPoint);
            double distanceFromEnd = point.distance(endPoint);
            assertTrue("Point " + i + " = " + point + " is too close to start " + startPoint + ", with a distance of " + distanceFromStart
                             + ", which should be at least " + distanceToFilter, distanceFromStart > distanceToFilter);
            assertTrue(
                  "Point " + i + " = " + point + " is too close to end " + endPoint + ", with a distance of " + distanceFromEnd + ", which should be at least "
                        + distanceToFilter, distanceFromEnd > distanceToFilter);
         }
      }
   }

   @Test
   public void testIsNearCliffAtCorner()
   {
      List<PlanarRegion> planarRegions = TestEnvironmentTools.createCornerEnvironment();
      PlanarRegionsList planarRegionsList = new PlanarRegionsList(planarRegions);
      DefaultVisibilityGraphParameters parameters = new DefaultVisibilityGraphParameters();

      List<NavigableRegion> navigableRegions = NavigableRegionsFactory.createNavigableRegions(planarRegions, parameters);


      double extrusionDistance = parameters.getNavigableExtrusionDistance();
      double maxInterRegionConnectionLength = parameters.getMaxInterRegionConnectionLength();
      double desiredDistanceFromCliff = parameters.getPreferredObstacleExtrusionDistance();
      double cliffHeightToAvoid = 0.10;

      Point2D pointOnCorner = new Point2D(5.0, 2.5);
      pointOnCorner.subX(extrusionDistance);
      pointOnCorner.subY(extrusionDistance);

      NavigableRegion homeRegion = getRegionContainingPoint(pointOnCorner, navigableRegions);

      List<LineSegment2DReadOnly> cliffEdgesToPack = new ArrayList<>();
      boolean isNearCliff = ObstacleAndCliffAvoidanceProcessor.isNearCliff(pointOnCorner, maxInterRegionConnectionLength, cliffHeightToAvoid, desiredDistanceFromCliff, homeRegion, navigableRegions, cliffEdgesToPack);

      assertTrue(isNearCliff);

      Point2D pointOnRightEdge = new Point2D(2.5, 2.5);
      pointOnCorner.subY(extrusionDistance);

      cliffEdgesToPack.clear();
      isNearCliff = ObstacleAndCliffAvoidanceProcessor.isNearCliff(pointOnRightEdge, maxInterRegionConnectionLength, cliffHeightToAvoid, desiredDistanceFromCliff, homeRegion, navigableRegions, cliffEdgesToPack);

      assertTrue(isNearCliff);


      Point2D pointOnLeftEdge = new Point2D(2.5, -2.5);
      pointOnCorner.addY(extrusionDistance);

      cliffEdgesToPack.clear();
      isNearCliff = ObstacleAndCliffAvoidanceProcessor.isNearCliff(pointOnLeftEdge, maxInterRegionConnectionLength, cliffHeightToAvoid, desiredDistanceFromCliff, homeRegion, navigableRegions, cliffEdgesToPack);

      assertTrue(isNearCliff);

      Point2D pointOnBackEdge = new Point2D(-5.0, 0.0);
      pointOnCorner.addX(extrusionDistance);

      cliffEdgesToPack.clear();
      isNearCliff = ObstacleAndCliffAvoidanceProcessor.isNearCliff(pointOnBackEdge, maxInterRegionConnectionLength, cliffHeightToAvoid, desiredDistanceFromCliff, homeRegion, navigableRegions, cliffEdgesToPack);

      assertTrue(isNearCliff);


      Point2D pointOnFrontEdge = new Point2D(5.0, 0.0);
      pointOnCorner.subX(extrusionDistance);

      cliffEdgesToPack.clear();
      isNearCliff = ObstacleAndCliffAvoidanceProcessor.isNearCliff(pointOnFrontEdge, maxInterRegionConnectionLength, cliffHeightToAvoid, desiredDistanceFromCliff, homeRegion, navigableRegions, cliffEdgesToPack);

      assertFalse(isNearCliff);

      // TODO this one is a little trickier. The node is on a shared edge, the front one, but is not far enough from a cliff edge. currently fails.
      // set it near a little closer to the edge than the preferred extrusion distance
      pointOnFrontEdge.setY(-2.5);
      pointOnFrontEdge.addY(parameters.getPreferredObstacleExtrusionDistance());
      pointOnFrontEdge.subY(0.05);

      cliffEdgesToPack.clear();
      isNearCliff = ObstacleAndCliffAvoidanceProcessor.isNearCliff(pointOnFrontEdge, maxInterRegionConnectionLength, cliffHeightToAvoid, desiredDistanceFromCliff, homeRegion, navigableRegions, cliffEdgesToPack);

      assertTrue(isNearCliff);



      if (visualize)
         visualize(planarRegionsList, pointOnCorner, cliffEdgesToPack, parameters, navigableRegions);

   }

   @Test
   public void testIsNearCliffUsingParallelRegions()
   {
      List<PlanarRegion> planarRegions = TestEnvironmentTools.createSameEdgeRegions();
      PlanarRegionsList planarRegionsList = new PlanarRegionsList(planarRegions);

      DefaultVisibilityGraphParameters parameters = new DefaultVisibilityGraphParameters();

      List<NavigableRegion> navigableRegions = NavigableRegionsFactory.createNavigableRegions(planarRegions, parameters);


      double extrusionDistance = parameters.getNavigableExtrusionDistance();
      double maxInterRegionConnectionLength = parameters.getMaxInterRegionConnectionLength();
      double desiredDistanceFromCliff = parameters.getPreferredObstacleExtrusionDistance();
      double cliffHeightToAvoid = 0.10;


      Point2D pointOnOutsideSharedEdge = new Point2D(1.0, 1.0);
      pointOnOutsideSharedEdge.subX(extrusionDistance);
      pointOnOutsideSharedEdge.subY(extrusionDistance);
      NavigableRegion homeRegion = getRegionContainingPoint(pointOnOutsideSharedEdge, navigableRegions);

      List<LineSegment2DReadOnly> cliffEdgesToPack = new ArrayList<>();

      cliffEdgesToPack.clear();
      boolean isNearCliff = ObstacleAndCliffAvoidanceProcessor.isNearCliff(pointOnOutsideSharedEdge, maxInterRegionConnectionLength, cliffHeightToAvoid, desiredDistanceFromCliff, homeRegion, navigableRegions, cliffEdgesToPack);

      boolean expected = true;

      if (visualize && expected != isNearCliff)
         visualize(planarRegionsList, pointOnOutsideSharedEdge, cliffEdgesToPack, parameters, navigableRegions);

      assertEquals(expected, isNearCliff);

      pointOnOutsideSharedEdge.setY(1.0);
      pointOnOutsideSharedEdge.subY(parameters.getPreferredObstacleExtrusionDistance());
      pointOnOutsideSharedEdge.addY(0.1);

      cliffEdgesToPack.clear();
      isNearCliff = ObstacleAndCliffAvoidanceProcessor.isNearCliff(pointOnOutsideSharedEdge, maxInterRegionConnectionLength, cliffHeightToAvoid, desiredDistanceFromCliff, homeRegion, navigableRegions, cliffEdgesToPack);

      expected = true;

      if (visualize && expected != isNearCliff)
         visualize(planarRegionsList, pointOnOutsideSharedEdge, cliffEdgesToPack, parameters, navigableRegions);

      assertEquals(expected, isNearCliff);



      pointOnOutsideSharedEdge.setX(0.0);

      cliffEdgesToPack.clear();
      isNearCliff = ObstacleAndCliffAvoidanceProcessor.isNearCliff(pointOnOutsideSharedEdge, maxInterRegionConnectionLength, cliffHeightToAvoid, desiredDistanceFromCliff, homeRegion, navigableRegions, cliffEdgesToPack);

      expected = true;

      if (visualize && expected != isNearCliff)
         visualize(planarRegionsList, pointOnOutsideSharedEdge, cliffEdgesToPack, parameters, navigableRegions);

      assertEquals(expected, isNearCliff);
   }

   @Test
   public void testTrickyCase()
   {
      List<PlanarRegion> planarRegions = TestEnvironmentTools.createSameEdgeRegions();
      PlanarRegionsList planarRegionsList = new PlanarRegionsList(planarRegions);

      DefaultVisibilityGraphParameters parameters = new DefaultVisibilityGraphParameters();

      List<NavigableRegion> navigableRegions = NavigableRegionsFactory.createNavigableRegions(planarRegions, parameters);


      double extrusionDistance = parameters.getNavigableExtrusionDistance();
      double maxInterRegionConnectionLength = parameters.getMaxInterRegionConnectionLength();
      double desiredDistanceFromCliff = parameters.getPreferredObstacleExtrusionDistance();
      double cliffHeightToAvoid = 0.10;


      Point2D pointOnOutsideSharedEdge = new Point2D(1.0, 1.0);
      pointOnOutsideSharedEdge.subX(extrusionDistance);
      pointOnOutsideSharedEdge.subY(extrusionDistance);
      NavigableRegion homeRegion = getRegionContainingPoint(pointOnOutsideSharedEdge, navigableRegions);

      List<LineSegment2DReadOnly> cliffEdgesToPack = new ArrayList<>();


      pointOnOutsideSharedEdge.setY(1.0);
      pointOnOutsideSharedEdge.subY(parameters.getPreferredObstacleExtrusionDistance());
      pointOnOutsideSharedEdge.addY(0.1);

      cliffEdgesToPack.clear();
      boolean isNearCliff = ObstacleAndCliffAvoidanceProcessor.isNearCliff(pointOnOutsideSharedEdge, maxInterRegionConnectionLength, cliffHeightToAvoid, desiredDistanceFromCliff, homeRegion, navigableRegions, cliffEdgesToPack);

      boolean expected = true;

      if (visualize && expected != isNearCliff)
      {
         visualize(planarRegionsList, pointOnOutsideSharedEdge, cliffEdgesToPack, parameters, navigableRegions);
      }

      assertEquals(expected, isNearCliff);

   }

   @Test
   public void testTrickyCase2()
   {
      List<PlanarRegion> planarRegions = TestEnvironmentTools.createBigToSmallRegions();
      PlanarRegionsList planarRegionsList = new PlanarRegionsList(planarRegions);

      DefaultVisibilityGraphParameters parameters = new DefaultVisibilityGraphParameters();

      List<NavigableRegion> navigableRegions = NavigableRegionsFactory.createNavigableRegions(planarRegions, parameters);


      double extrusionDistance = parameters.getNavigableExtrusionDistance();
      double maxInterRegionConnectionLength = parameters.getMaxInterRegionConnectionLength();
      double desiredDistanceFromCliff = parameters.getPreferredObstacleExtrusionDistance();
      double cliffHeightToAvoid = 0.10;


      Point2D pointOnOutsideSharedEdge = new Point2D(5.0, 0.0);
      pointOnOutsideSharedEdge.subX(extrusionDistance);
      NavigableRegion homeRegion = getRegionContainingPoint(pointOnOutsideSharedEdge, navigableRegions);

      List<LineSegment2DReadOnly> cliffEdgesToPack = new ArrayList<>();

      cliffEdgesToPack.clear();
      boolean isNearCliff = ObstacleAndCliffAvoidanceProcessor.isNearCliff(pointOnOutsideSharedEdge, maxInterRegionConnectionLength, cliffHeightToAvoid, desiredDistanceFromCliff, homeRegion, navigableRegions, cliffEdgesToPack);

      boolean expected = false;

      if (visualize && expected != isNearCliff)
      {
         visualize(planarRegionsList, pointOnOutsideSharedEdge, cliffEdgesToPack, parameters, navigableRegions);
      }

      assertEquals(expected, isNearCliff);

   }

   private static NavigableRegion getRegionContainingPoint(Point2DReadOnly point, List<NavigableRegion> navigableRegions)
   {
      NavigableRegion homeRegion = null;
      for (int i = 0; i < navigableRegions.size(); i++)
      {
         Cluster homeCluster = navigableRegions.get(i).getHomeRegionCluster();
         if (EuclidGeometryPolygonTools.isPoint2DInsideConvexPolygon2D(point, homeCluster.getRawPointsInWorld2D(), homeCluster.getNumberOfRawPoints(), true, 0.0));
         {
            homeRegion = navigableRegions.get(i);
            break;
         }
      }

      return homeRegion;
   }

   private void visualize(PlanarRegionsList planarRegionsList, Point2DReadOnly point, List<LineSegment2DReadOnly> cliffEdges, VisibilityGraphsParametersBasics parameters, List<NavigableRegion> navigableRegions)
   {
      double maxInterRegionConnectionLength = parameters.getMaxInterRegionConnectionLength();
      double desiredDistanceFromCliff = parameters.getPreferredObstacleExtrusionDistance();
      double cliffHeightToAvoid = 0.10;

      NavigableRegion homeRegion = getRegionContainingPoint(point, navigableRegions);

      List<LineSegment2DReadOnly> homeEdges = ObstacleAndCliffAvoidanceProcessor.getNearbyEdges(point, homeRegion.getHomeRegionCluster(), desiredDistanceFromCliff);

      List<NavigableRegion> nearbyRegions = ObstacleAndCliffAvoidanceProcessor.filterNavigableRegionsWithBoundingCircle(point, maxInterRegionConnectionLength + desiredDistanceFromCliff, navigableRegions);
      List<NavigableRegion> closeEnoughRegions = ObstacleAndCliffAvoidanceProcessor.filterNavigableRegionsConnectionWithDistanceAndHeightChange(homeRegion, nearbyRegions, maxInterRegionConnectionLength,
                                                                                                                                                cliffHeightToAvoid);
      List<LineSegment2DReadOnly> nearbyEdges = new ArrayList<>();

      for (NavigableRegion closeEnoughRegion : closeEnoughRegions)
      {
         nearbyEdges.addAll(ObstacleAndCliffAvoidanceProcessor.getNearbyEdges(point, closeEnoughRegion.getHomeRegionCluster(), desiredDistanceFromCliff));
      }


      SimulationConstructionSet scs = new SimulationConstructionSet();
      scs.setGroundVisible(false);
      PlanarRegionsListDefinedEnvironment environment = new PlanarRegionsListDefinedEnvironment(planarRegionsList, 0.0, false);

      scs.addStaticLinkGraphics(environment.getTerrainObject3D().getLinkGraphics());

      Graphics3DObject extraGraphics = new Graphics3DObject();
      extraGraphics.translate(new Point3D(point));
      extraGraphics.addSphere(0.05, YoAppearance.Red());

      for (LineSegment2DReadOnly edge : homeEdges)
      {
         extraGraphics.identity();
         extraGraphics.translate(new Point3D(edge.getFirstEndpoint()));
         extraGraphics.addSphere(0.03, YoAppearance.Black());

         extraGraphics.identity();
         extraGraphics.translate(new Point3D(edge.getSecondEndpoint()));
         extraGraphics.addSphere(0.03, YoAppearance.Black());
      }

      for (LineSegment2DReadOnly edge : nearbyEdges)
      {
         extraGraphics.identity();
         extraGraphics.translate(new Point3D(edge.getFirstEndpoint()));
         extraGraphics.addSphere(0.03, YoAppearance.Green());

         extraGraphics.identity();
         extraGraphics.translate(new Point3D(edge.getSecondEndpoint()));
         extraGraphics.addSphere(0.03, YoAppearance.Green());
      }

      for (LineSegment2DReadOnly edge : cliffEdges)
      {
         extraGraphics.identity();
         extraGraphics.translate(new Point3D(edge.getFirstEndpoint()));
         extraGraphics.addSphere(0.03, YoAppearance.Blue());

         extraGraphics.identity();
         extraGraphics.translate(new Point3D(edge.getSecondEndpoint()));
         extraGraphics.addSphere(0.03, YoAppearance.Blue());

      }

      scs.addStaticLinkGraphics(extraGraphics);


      scs.startOnAThread();
      ThreadTools.sleepForever();
   }
}

