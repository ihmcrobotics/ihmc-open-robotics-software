package us.ihmc.pathPlanning.visibilityGraphs.tools;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import org.junit.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.ClusterType;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.ExtrusionSide;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.Connection;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.ConnectionPoint3D;
import us.ihmc.robotics.geometry.PlanarRegion;

public class VisibilityToolsTest
{
   private static final double EPSILON = 1.0e-12;

   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   public void testIsPointVisibleForStaticMaps()
   {
      Cluster keepOutClusterOne = new Cluster(ExtrusionSide.OUTSIDE, ClusterType.POLYGON);
      keepOutClusterOne.addNonNavigableExtrusionInLocal(new Point2D(-0.1, 0.5));
      keepOutClusterOne.addNonNavigableExtrusionInLocal(new Point2D(1.1, 0.5));

      Cluster keepOutClusterTwo = new Cluster(ExtrusionSide.OUTSIDE, ClusterType.POLYGON);
      keepOutClusterTwo.addNonNavigableExtrusionInLocal(new Point2D(2.5, -0.1));
      keepOutClusterTwo.addNonNavigableExtrusionInLocal(new Point2D(2.5, 1.1));

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

      assertTrue(VisibilityTools.isPointVisibleForStaticMaps(clusters, pointA, pointB));
      assertFalse(VisibilityTools.isPointVisibleForStaticMaps(clusters, pointA, pointC));
      assertFalse(VisibilityTools.isPointVisibleForStaticMaps(clusters, pointA, pointD));

      assertTrue(VisibilityTools.isPointVisibleForStaticMaps(clusters, pointA, pointE));
      assertFalse(VisibilityTools.isPointVisibleForStaticMaps(clusters, pointA, pointF));
      assertFalse(VisibilityTools.isPointVisibleForStaticMaps(clusters, pointA, pointG));
      assertFalse(VisibilityTools.isPointVisibleForStaticMaps(clusters, pointA, pointH));

      assertTrue(VisibilityTools.isPointVisibleForStaticMaps(clusters, pointB, pointA));
      assertFalse(VisibilityTools.isPointVisibleForStaticMaps(clusters, pointB, pointC));
      assertFalse(VisibilityTools.isPointVisibleForStaticMaps(clusters, pointB, pointD));

      assertTrue(VisibilityTools.isPointVisibleForStaticMaps(clusters, pointB, pointE));
      assertFalse(VisibilityTools.isPointVisibleForStaticMaps(clusters, pointB, pointF));
      assertFalse(VisibilityTools.isPointVisibleForStaticMaps(clusters, pointB, pointG));
      assertTrue(VisibilityTools.isPointVisibleForStaticMaps(clusters, pointB, pointH));

      assertFalse(VisibilityTools.isPointVisibleForStaticMaps(clusters, pointC, pointA));
      assertFalse(VisibilityTools.isPointVisibleForStaticMaps(clusters, pointC, pointB));
      assertTrue(VisibilityTools.isPointVisibleForStaticMaps(clusters, pointC, pointD));

      assertTrue(VisibilityTools.isPointVisibleForStaticMaps(clusters, pointC, pointE));
      assertFalse(VisibilityTools.isPointVisibleForStaticMaps(clusters, pointC, pointF));
      assertFalse(VisibilityTools.isPointVisibleForStaticMaps(clusters, pointC, pointG));
      assertTrue(VisibilityTools.isPointVisibleForStaticMaps(clusters, pointC, pointH));
   }

   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
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

      Point2DReadOnly closestPointOnAboveHorizontalLineExpected = new Point2D(0.0, 0.2);
      Point2DReadOnly closestPointOnAboveHorizontalClusterExpected = new Point2D(0.0, 0.1);

      distance = VisibilityTools.distanceToCluster(firstPointAboveHorizontal, secondPointAboveHorizontal, pointsInCluster, closestPointOnAboveHorizontalLine,
                                                   closestPointOnAboveHorizontalCluster, null, true);

      assertEquals(0.1, distance, EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(closestPointOnAboveHorizontalClusterExpected, closestPointOnAboveHorizontalCluster, EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(closestPointOnAboveHorizontalLineExpected, closestPointOnAboveHorizontalLine, EPSILON);

   }

   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   public void testIsPointVisibleForStaticMapsClosedPolygonVsOpenMultiLine()
   {
      Cluster keepOutClusterPolygon = new Cluster(ExtrusionSide.OUTSIDE, ClusterType.POLYGON);
      keepOutClusterPolygon.addNonNavigableExtrusionInLocal(new Point2D(0.0, 0.0));
      keepOutClusterPolygon.addNonNavigableExtrusionInLocal(new Point2D(0.0, 1.0));
      keepOutClusterPolygon.addNonNavigableExtrusionInLocal(new Point2D(1.0, 1.0));
      keepOutClusterPolygon.addNonNavigableExtrusionInLocal(new Point2D(1.0, 0.0));

      Cluster keepOutClusterMultiline = new Cluster(ExtrusionSide.INSIDE, ClusterType.MULTI_LINE);
      keepOutClusterMultiline.addNonNavigableExtrusionInLocal(new Point2D(0.0, 0.0));
      keepOutClusterMultiline.addNonNavigableExtrusionInLocal(new Point2D(0.0, 1.0));
      keepOutClusterMultiline.addNonNavigableExtrusionInLocal(new Point2D(1.0, 1.0));
      keepOutClusterMultiline.addNonNavigableExtrusionInLocal(new Point2D(1.0, 0.0));

      Point2D pointOutside = new Point2D(0.5, -1.0);
      Point2D pointInside = new Point2D(0.5, 0.5);

      ArrayList<Cluster> polygonClusters = new ArrayList<>();
      polygonClusters.add(keepOutClusterPolygon);
      assertFalse(VisibilityTools.isPointVisibleForStaticMaps(polygonClusters, pointOutside, pointInside));

      ArrayList<Cluster> multilineClusters = new ArrayList<>();
      multilineClusters.add(keepOutClusterMultiline);
      assertTrue(VisibilityTools.isPointVisibleForStaticMaps(multilineClusters, pointOutside, pointInside));
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
