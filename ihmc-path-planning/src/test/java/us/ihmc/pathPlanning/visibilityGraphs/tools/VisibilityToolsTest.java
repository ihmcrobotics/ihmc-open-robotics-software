package us.ihmc.pathPlanning.visibilityGraphs.tools;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.ClusterType;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.ExtrusionSide;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.ExtrusionHull;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.Connection;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.ConnectionPoint3D;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Random;

import static us.ihmc.robotics.Assert.*;

public class VisibilityToolsTest
{
   private static final double EPSILON = 1.0e-12;
   private static final int iters = 1000;

   @Test
   public void testIsPointVisibleForStaticMaps()
   {
      Cluster keepOutClusterOne = new Cluster(ExtrusionSide.OUTSIDE, ClusterType.POLYGON);
      ExtrusionHull clusterOne = new ExtrusionHull();
      clusterOne.addPoint(new Point2D(-0.1, 0.5));
      clusterOne.addPoint(new Point2D(1.1, 0.5));
      keepOutClusterOne.addNonNavigableExtrusionsInLocal(clusterOne);

      Cluster keepOutClusterTwo = new Cluster(ExtrusionSide.OUTSIDE, ClusterType.POLYGON);
      ExtrusionHull clusterTwo = new ExtrusionHull();
      clusterTwo.addPoint(new Point2D(2.5, -0.1));
      clusterTwo.addPoint(new Point2D(2.5, 1.1));
      keepOutClusterTwo.addNonNavigableExtrusionsInLocal(clusterTwo);

      List<Cluster> clusters = new ArrayList<>();
      clusters.add(keepOutClusterOne);
      clusters.add(keepOutClusterTwo);

      Point2D pointA = new Point2D(0.0, 0.0);
      Point2D pointB = new Point2D(1.0, 0.0);
      Point2D pointC = new Point2D(1.0, 1.0);
      Point2D pointD = new Point2D(0.0, 1.0);

      Point2D pointE = new Point2D(2.0, 0.0);
      Point2D pointF = new Point2D(3.0, 0.0);
      Point2D pointG = new Point2D(3.0, 1.0);
      Point2D pointH = new Point2D(2.0, 1.0);

      assertTrue(VisibilityTools.isPointVisibleToPointInSameRegion(clusters, pointA, pointB));
      assertFalse(VisibilityTools.isPointVisibleToPointInSameRegion(clusters, pointA, pointC));
      assertFalse(VisibilityTools.isPointVisibleToPointInSameRegion(clusters, pointA, pointD));

      assertTrue(VisibilityTools.isPointVisibleToPointInSameRegion(clusters, pointA, pointE));
      assertFalse(VisibilityTools.isPointVisibleToPointInSameRegion(clusters, pointA, pointF));
      assertFalse(VisibilityTools.isPointVisibleToPointInSameRegion(clusters, pointA, pointG));
      assertFalse(VisibilityTools.isPointVisibleToPointInSameRegion(clusters, pointA, pointH));

      assertTrue(VisibilityTools.isPointVisibleToPointInSameRegion(clusters, pointB, pointA));
      assertFalse(VisibilityTools.isPointVisibleToPointInSameRegion(clusters, pointB, pointC));
      assertFalse(VisibilityTools.isPointVisibleToPointInSameRegion(clusters, pointB, pointD));

      assertTrue(VisibilityTools.isPointVisibleToPointInSameRegion(clusters, pointB, pointE));
      assertFalse(VisibilityTools.isPointVisibleToPointInSameRegion(clusters, pointB, pointF));
      assertFalse(VisibilityTools.isPointVisibleToPointInSameRegion(clusters, pointB, pointG));
      assertTrue(VisibilityTools.isPointVisibleToPointInSameRegion(clusters, pointB, pointH));

      assertFalse(VisibilityTools.isPointVisibleToPointInSameRegion(clusters, pointC, pointA));
      assertFalse(VisibilityTools.isPointVisibleToPointInSameRegion(clusters, pointC, pointB));
      assertTrue(VisibilityTools.isPointVisibleToPointInSameRegion(clusters, pointC, pointD));

      assertTrue(VisibilityTools.isPointVisibleToPointInSameRegion(clusters, pointC, pointE));
      assertFalse(VisibilityTools.isPointVisibleToPointInSameRegion(clusters, pointC, pointF));
      assertFalse(VisibilityTools.isPointVisibleToPointInSameRegion(clusters, pointC, pointG));
      assertTrue(VisibilityTools.isPointVisibleToPointInSameRegion(clusters, pointC, pointH));
   }

   @Test
   public void testDistanceFromConnectionToCluster()
   {
      Point2D leftWallPoint = new Point2D(0.5, 0.0);
      Point2D rightWallPoint = new Point2D(-0.5, 0.0);


      double fortyFive = Math.sin(Math.PI / 4.0);
      List<Point2DReadOnly> pointsInCluster = new ArrayList<>();
      pointsInCluster.add(new Point2D(0.1 + 0.5, 0.0));
      pointsInCluster.add(new Point2D(0.1 * fortyFive + 0.5, 0.1 * fortyFive));
      pointsInCluster.add(new Point2D(0.5, 0.1));
      pointsInCluster.add(new Point2D(-0.5, 0.1));
      pointsInCluster.add(new Point2D(-0.1 * fortyFive - 0.5, 0.1 * fortyFive));
      pointsInCluster.add(new Point2D(-0.1 - 0.5, 0.0));
      pointsInCluster.add(new Point2D(-0.1 * fortyFive - 0.5, -0.1 * fortyFive));
      pointsInCluster.add(new Point2D(-0.5, -0.1));
      pointsInCluster.add(new Point2D(0.5, -0.1));
      pointsInCluster.add(new Point2D(0.1 * fortyFive + 0.5, -0.1 * fortyFive));


      // line to the left of the cluster
      Point2DReadOnly firstPointLeftVertical = new Point2D(0.7, -0.2);
      Point2DReadOnly secondPointLeftVertical = new Point2D(0.7, 0.2);

      Point2D closestPointOnLeftVerticalLine = new Point2D();
      Point2D closestPointOnLeftVerticalCluster = new Point2D();

      Point2DReadOnly closestPointOnLeftVerticalLineExpected = new Point2D(0.7, 0.0);
      Point2DReadOnly closestPointOnLeftVerticalClusterExpected = new Point2D(0.6, 0.0);

      double distance = VisibilityTools.distanceToCluster(firstPointLeftVertical, secondPointLeftVertical, pointsInCluster, closestPointOnLeftVerticalLine,
                                                          closestPointOnLeftVerticalCluster, null, true);

      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(closestPointOnLeftVerticalClusterExpected, closestPointOnLeftVerticalCluster, EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(closestPointOnLeftVerticalLineExpected, closestPointOnLeftVerticalLine, EPSILON);
      assertEquals(0.1, distance, EPSILON);


      // line to the right of the cluster
      Point2DReadOnly firstPointRightVertical = new Point2D(-0.7, -0.2);
      Point2DReadOnly secondPointRightVertical = new Point2D(-0.7, 0.2);

      Point2D closestPointOnRightVerticalLine = new Point2D();
      Point2D closestPointOnRightVerticalCluster = new Point2D();

      Point2DReadOnly closestPointOnRightVerticalLineExpected = new Point2D(-0.7, 0.0);
      Point2DReadOnly closestPointOnRightVerticalClusterExpected = new Point2D(-0.6, 0.0);

      distance = VisibilityTools.distanceToCluster(firstPointRightVertical, secondPointRightVertical, pointsInCluster, closestPointOnRightVerticalLine,
                                                          closestPointOnRightVerticalCluster, null, true);

      assertEquals(0.1, distance, EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(closestPointOnRightVerticalClusterExpected, closestPointOnRightVerticalCluster, EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(closestPointOnRightVerticalLineExpected, closestPointOnRightVerticalLine, EPSILON);

      // line above the cluster
      Point2DReadOnly firstPointAboveHorizontal = new Point2D(0.7, 0.2);
      Point2DReadOnly secondPointAboveHorizontal = new Point2D(-0.7, 0.2);

      Point2D closestPointOnAboveHorizontalLine = new Point2D();
      Point2D closestPointOnAboveHorizontalCluster = new Point2D();

      // goes to the first point because of clockwise motion
      Point2DReadOnly closestPointOnAboveHorizontalLineExpected = new Point2D(0.5, 0.2);
      Point2DReadOnly closestPointOnAboveHorizontalClusterExpected = new Point2D(0.5, 0.1);

      distance = VisibilityTools.distanceToCluster(firstPointAboveHorizontal, secondPointAboveHorizontal, pointsInCluster, closestPointOnAboveHorizontalLine,
                                                   closestPointOnAboveHorizontalCluster, null, true);

      assertEquals(0.1, distance, EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(closestPointOnAboveHorizontalClusterExpected, closestPointOnAboveHorizontalCluster, EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(closestPointOnAboveHorizontalLineExpected, closestPointOnAboveHorizontalLine, EPSILON);

   }


   @Test
   public void testIsPointVisibleTroublingCase()
   {
      List<Point2DReadOnly> listOfPointsInCluster = new ArrayList<>();
      listOfPointsInCluster.add(new Point2D(0.101, 0.5));
      listOfPointsInCluster.add(new Point2D(0.115, 0.535));
      listOfPointsInCluster.add(new Point2D(0.150, 0.549));
      listOfPointsInCluster.add(new Point2D(0.450, 0.549));
      listOfPointsInCluster.add(new Point2D(0.485, 0.535));
      listOfPointsInCluster.add(new Point2D(0.499, 0.5));
      listOfPointsInCluster.add(new Point2D(0.499, -0.5));
      listOfPointsInCluster.add(new Point2D(0.485, -0.535));
      listOfPointsInCluster.add(new Point2D(0.45, -0.549));
      listOfPointsInCluster.add(new Point2D(0.15, -0.549));
      listOfPointsInCluster.add(new Point2D(0.115, -0.535));
      listOfPointsInCluster.add(new Point2D(0.101, -0.5));

      Point2DReadOnly observer = new Point2D(-0.005, 0.0);
      Point2DReadOnly target = new Point2D(0.131, 0.0);

      assertFalse(VisibilityTools.isPointVisible(observer, target, listOfPointsInCluster, true));


      listOfPointsInCluster = new ArrayList<>();
      listOfPointsInCluster.add(new Point2D(0.10099999999999998, 0.5));
      listOfPointsInCluster.add(new Point2D(0.11535176772185912, 0.5346482322781408));
      listOfPointsInCluster.add(new Point2D(0.14999999999999997, 0.549));
      listOfPointsInCluster.add(new Point2D(0.450, 0.549));
      listOfPointsInCluster.add(new Point2D(0.48464823227814086, 0.5346482322781408));
      listOfPointsInCluster.add(new Point2D(0.49900000000000005, 0.5));
      listOfPointsInCluster.add(new Point2D(0.49900000000000005, -0.5));
      listOfPointsInCluster.add(new Point2D(0.48464823227814086, -0.5346482322781408));
      listOfPointsInCluster.add(new Point2D(0.45, -0.549));
      listOfPointsInCluster.add(new Point2D(0.14999999999999997, -0.549));
      listOfPointsInCluster.add(new Point2D(0.11535176772185912, -0.5346482322781408));
      listOfPointsInCluster.add(new Point2D(0.10099999999999998, -0.5));

      observer = new Point2D(-0.0047499999999999765, 0.0);
      target = new Point2D(0.1305, 0.0);

      assertFalse(VisibilityTools.isPointVisible(observer, target, listOfPointsInCluster, true));
   }

   @Test
   public void testIsPointVisibleForStaticMapsClosedPolygonVsOpenMultiLine()
   {
      Cluster keepOutClusterPolygon = new Cluster(ExtrusionSide.OUTSIDE, ClusterType.POLYGON);
      ExtrusionHull polygonPoints = new ExtrusionHull();
      polygonPoints.addPoint(new Point2D(0.0, 0.0));
      polygonPoints.addPoint(new Point2D(0.0, 1.0));
      polygonPoints.addPoint(new Point2D(1.0, 1.0));
      polygonPoints.addPoint(new Point2D(1.0, 0.0));
      keepOutClusterPolygon.addNonNavigableExtrusionsInLocal(polygonPoints);

      Cluster keepOutClusterMultiline = new Cluster(ExtrusionSide.INSIDE, ClusterType.MULTI_LINE);
      ExtrusionHull multilinePoints = new ExtrusionHull();
      multilinePoints.addPoint(new Point2D(0.0, 0.0));
      multilinePoints.addPoint(new Point2D(0.0, 1.0));
      multilinePoints.addPoint(new Point2D(1.0, 1.0));
      multilinePoints.addPoint(new Point2D(1.0, 0.0));
      keepOutClusterMultiline.addNonNavigableExtrusionsInLocal(multilinePoints);

      Point2D pointOutside = new Point2D(0.5, -1.0);
      Point2D pointInside = new Point2D(0.5, 0.5);

      ArrayList<Cluster> polygonClusters = new ArrayList<>();
      polygonClusters.add(keepOutClusterPolygon);
      assertFalse(VisibilityTools.isPointVisibleToPointInSameRegion(polygonClusters, pointOutside, pointInside));

      ArrayList<Cluster> multilineClusters = new ArrayList<>();
      multilineClusters.add(keepOutClusterMultiline);
      assertTrue(VisibilityTools.isPointVisibleToPointInSameRegion(multilineClusters, pointOutside, pointInside));
   }


   private PlanarRegion createAHomeRegionSquare(double xyMax)
   {
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      ConvexPolygon2D homeRegionPolygon = new ConvexPolygon2D();
      homeRegionPolygon.addVertex(-xyMax, -xyMax);
      homeRegionPolygon.addVertex(xyMax, -xyMax);
      homeRegionPolygon.addVertex(xyMax, xyMax);
      homeRegionPolygon.addVertex(-xyMax, xyMax);
      homeRegionPolygon.update();
      PlanarRegion homeRegion = new PlanarRegion(transformToWorld, homeRegionPolygon);
      return homeRegion;
   }

   private void assertConnectionEquals(Point2D expectedSourcePoint, Point2D expectedTargetPoint, Connection connection)
   {
      ConnectionPoint3D sourcePoint = connection.getSourcePoint();
      ConnectionPoint3D targetPoint = connection.getTargetPoint();

      assertPointEquals(expectedSourcePoint, sourcePoint);
      assertPointEquals(expectedTargetPoint, targetPoint);
   }

   private void assertPointEquals(Point2D expectedPoint, ConnectionPoint3D point)
   {
      assertEquals(expectedPoint.getX(), point.getX(), EPSILON);
      assertEquals(expectedPoint.getY(), point.getY(), EPSILON);
   }

   private void printConnections(Collection<Connection> connections)
   {
      //      System.out.print("{");

      for (Connection connection : connections)
      {
         System.out.println(connection);

         //         System.out.print("{" + connection.getSourcePoint().getX() + ", " + connection.getSourcePoint().getY() + "}" + ",");
      }
      //      System.out.println("}");

   }



   public static void main(String[] args) throws IOException
   {
      MutationTestFacilitator.facilitateMutationTestForClass(VisibilityTools.class, VisibilityToolsTest.class);
   }

}
