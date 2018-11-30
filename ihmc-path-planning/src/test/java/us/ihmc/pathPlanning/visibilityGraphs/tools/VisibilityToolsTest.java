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
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.Connection;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.ConnectionPoint3D;
import us.ihmc.robotics.geometry.PlanarRegion;

public class VisibilityToolsTest
{
   private static final double EPSILON = 1.0e-12;

   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   public void testAddClusterSelfVisibilityOneJustASquare()
   {
      PlanarRegion homeRegion = createAHomeRegionSquare(5.0);

      // Put a simple square extrusion in the middle of it.
      Cluster clusterToBuildMapOf = new Cluster();
      clusterToBuildMapOf.addNavigableExtrusionInLocal(new Point2D(0.0, 0.0));
      clusterToBuildMapOf.addNavigableExtrusionInLocal(new Point2D(1.0, 0.0));
      clusterToBuildMapOf.addNavigableExtrusionInLocal(new Point2D(1.0, 1.0));
      clusterToBuildMapOf.addNavigableExtrusionInLocal(new Point2D(0.0, 1.0));

      List<Cluster> allClusters = new ArrayList<>();
      allClusters.add(clusterToBuildMapOf);

      int mapId = 77;
      ArrayList<Connection> connections = new ArrayList<>();
      VisibilityTools.addClusterSelfVisibility(clusterToBuildMapOf, homeRegion, allClusters, mapId, connections);

      Point2D pointA = new Point2D(0.0, 0.0);
      Point2D pointB = new Point2D(1.0, 0.0);
      Point2D pointC = new Point2D(1.0, 1.0);
      Point2D pointD = new Point2D(0.0, 1.0);

      int index = 0;

      assertConnectionEquals(pointA, pointB, connections.get(index++));
      assertConnectionEquals(pointA, pointC, connections.get(index++));
      assertConnectionEquals(pointA, pointD, connections.get(index++));
      assertConnectionEquals(pointB, pointC, connections.get(index++));
      assertConnectionEquals(pointB, pointD, connections.get(index++));
      assertConnectionEquals(pointC, pointD, connections.get(index++));

      assertEquals(6, connections.size());
   }

   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   public void testAddClusterSelfVisibilityTwoAnExtrudedLine()
   {
      // Make a square home region polygon.
      PlanarRegion homeRegion = createAHomeRegionSquare(1.0);

      // Put an extrusion in the middle of it.
      Point2D endpoint1 = new Point2D(-0.5, 0.0);
      Point2D endpoint2 = new Point2D(0.5, 0.0);
      double extrusionDistance = 0.1;

      List<Point2D> extrusions = ClusterTools.extrudeLine(endpoint1, endpoint2, extrusionDistance, 3);
      Cluster clusterToBuildMapOf = new Cluster();
      clusterToBuildMapOf.addNavigableExtrusionsInLocal(extrusions);

      List<Cluster> allClusters = new ArrayList<>();
      allClusters.add(clusterToBuildMapOf);

      int mapId = 77;
      ArrayList<Connection> connections = new ArrayList<>();
      VisibilityTools.addClusterSelfVisibility(clusterToBuildMapOf, homeRegion, allClusters, mapId, connections);

      Point2D pointA = new Point2D(-0.5, -0.1);
      Point2D pointB = new Point2D(-0.6, 0.0);
      Point2D pointC = new Point2D(-0.5, 0.1);
      Point2D pointD = new Point2D(0.5, 0.1);
      Point2D pointE = new Point2D(0.6, 0.0);
      Point2D pointF = new Point2D(0.5, -0.1);

      int index = 0;

      assertConnectionEquals(pointA, pointB, connections.get(index++));
      assertConnectionEquals(pointA, pointC, connections.get(index++));
      assertConnectionEquals(pointA, pointD, connections.get(index++));
      assertConnectionEquals(pointA, pointE, connections.get(index++));
      assertConnectionEquals(pointA, pointF, connections.get(index++));
      assertConnectionEquals(pointB, pointC, connections.get(index++));
      assertConnectionEquals(pointB, pointD, connections.get(index++));
      assertConnectionEquals(pointB, pointE, connections.get(index++));
      assertConnectionEquals(pointB, pointF, connections.get(index++));
      assertConnectionEquals(pointC, pointD, connections.get(index++));
      assertConnectionEquals(pointC, pointE, connections.get(index++));
      assertConnectionEquals(pointC, pointF, connections.get(index++));
      assertConnectionEquals(pointD, pointE, connections.get(index++));
      assertConnectionEquals(pointD, pointF, connections.get(index++));
      assertConnectionEquals(pointE, pointF, connections.get(index++));

      assertEquals(15, connections.size());
   }

   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   public void testAddClusterSelfVisibilityThreeSquareWithLineObstacle()
   {
      PlanarRegion homeRegion = createAHomeRegionSquare(5.0);

      Point2D pointA = new Point2D(0.0, 0.0);
      Point2D pointB = new Point2D(1.0, 0.0);
      Point2D pointC = new Point2D(1.0, 1.0);
      Point2D pointD = new Point2D(0.0, 1.0);

      // Put a simple square extrusion in the middle of it.
      Cluster clusterToBuildMapOf = new Cluster();
      clusterToBuildMapOf.addNavigableExtrusionInLocal(pointA);
      clusterToBuildMapOf.addNavigableExtrusionInLocal(pointB);
      clusterToBuildMapOf.addNavigableExtrusionInLocal(pointC);
      clusterToBuildMapOf.addNavigableExtrusionInLocal(pointD);

      Cluster obstacleCluster = new Cluster();
      obstacleCluster.addNonNavigableExtrusionInLocal(new Point2D(0.1, -0.1));
      obstacleCluster.addNonNavigableExtrusionInLocal(new Point2D(1.1, 0.9));

      List<Cluster> allClusters = new ArrayList<>();
      allClusters.add(clusterToBuildMapOf);
      allClusters.add(obstacleCluster);

      int mapId = 77;
      ArrayList<Connection> connections = new ArrayList<>();
      VisibilityTools.addClusterSelfVisibility(clusterToBuildMapOf, homeRegion, allClusters, mapId, connections);

      int index = 0;

      assertConnectionEquals(pointA, pointC, connections.get(index++));
      assertConnectionEquals(pointA, pointD, connections.get(index++));
      assertConnectionEquals(pointC, pointD, connections.get(index++));

      assertEquals(3, connections.size());
   }

   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   public void testAddClusterSelfVisibilityFourOverlappingSquares()
   {
      PlanarRegion homeRegion = createAHomeRegionSquare(5.0);

      Point2D pointA = new Point2D(0.0, 0.0);
      Point2D pointB = new Point2D(1.0, 0.0);
      Point2D pointC = new Point2D(1.0, 1.0);
      Point2D pointD = new Point2D(0.0, 1.0);

      // Put a simple square extrusion in the middle of it.
      Cluster clusterToBuildMapOf = new Cluster();
      clusterToBuildMapOf.addNavigableExtrusionInLocal(pointA);
      clusterToBuildMapOf.addNavigableExtrusionInLocal(pointB);
      clusterToBuildMapOf.addNavigableExtrusionInLocal(pointC);
      clusterToBuildMapOf.addNavigableExtrusionInLocal(pointD);

      Cluster obstacleCluster = new Cluster();

      obstacleCluster.addNonNavigableExtrusionInLocal(new Point2D(0.1, -0.1));
      obstacleCluster.addNonNavigableExtrusionInLocal(new Point2D(1.1, -0.1));
      obstacleCluster.addNonNavigableExtrusionInLocal(new Point2D(1.1, 1.1));
      obstacleCluster.addNonNavigableExtrusionInLocal(new Point2D(0.1, 1.1));

      assertFalse(obstacleCluster.isInsideNonNavigableZone(pointA));
      assertTrue(obstacleCluster.isInsideNonNavigableZone(pointB));
      assertTrue(obstacleCluster.isInsideNonNavigableZone(pointC));
      assertFalse(obstacleCluster.isInsideNonNavigableZone(pointD));

      List<Cluster> allClusters = new ArrayList<>();
      allClusters.add(clusterToBuildMapOf);
      allClusters.add(obstacleCluster);

      int mapId = 77;
      ArrayList<Connection> connections = new ArrayList<>();
      boolean[] arePointsNavigable = VisibilityTools.addClusterSelfVisibility(clusterToBuildMapOf, homeRegion, allClusters, mapId, connections);

      assertEquals(4, arePointsNavigable.length);

      assertTrue(arePointsNavigable[0]);
      assertFalse(arePointsNavigable[1]);
      assertFalse(arePointsNavigable[2]);
      assertTrue(arePointsNavigable[3]);

      int index = 0;

      assertConnectionEquals(pointA, pointD, connections.get(index++));
      assertEquals(1, connections.size());
   }

   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   public void testAddCrossClusterVisibilityOneSomeSquares()
   {
      Point2D pointA = new Point2D(0.0, 0.0);
      Point2D pointB = new Point2D(1.0, 0.0);
      Point2D pointC = new Point2D(1.0, 1.0);
      Point2D pointD = new Point2D(0.0, 1.0);

      Point2D pointE = new Point2D(2.0, 0.0);
      Point2D pointF = new Point2D(3.0, 0.0);
      Point2D pointG = new Point2D(3.0, 1.0);
      Point2D pointH = new Point2D(2.0, 1.0);

      // Put a simple square extrusion in the middle of it.
      Cluster clusterOne = new Cluster();

      clusterOne.addNavigableExtrusionInLocal(pointA);
      clusterOne.addNavigableExtrusionInLocal(pointB);
      clusterOne.addNavigableExtrusionInLocal(pointC);
      clusterOne.addNavigableExtrusionInLocal(pointD);

      Cluster clusterTwo = new Cluster();
      clusterTwo.addNavigableExtrusionInLocal(pointE);
      clusterTwo.addNavigableExtrusionInLocal(pointF);
      clusterTwo.addNavigableExtrusionInLocal(pointG);
      clusterTwo.addNavigableExtrusionInLocal(pointH);

      List<Cluster> allClusters = new ArrayList<>();
      allClusters.add(clusterOne);
      allClusters.add(clusterTwo);

      int mapId = 77;
      ArrayList<Connection> connections = new ArrayList<>();

      boolean[] sourceNavigability = new boolean[] {true, true, false, true};
      boolean[] targetNavigability = new boolean[] {false, true, true, true};

      VisibilityTools.addCrossClusterVisibility(clusterOne, sourceNavigability, clusterTwo, targetNavigability, allClusters, mapId, connections);

      int index = 0;

      assertConnectionEquals(pointA, pointF, connections.get(index++));
      assertConnectionEquals(pointA, pointG, connections.get(index++));
      assertConnectionEquals(pointA, pointH, connections.get(index++));

      assertConnectionEquals(pointB, pointF, connections.get(index++));
      assertConnectionEquals(pointB, pointG, connections.get(index++));
      assertConnectionEquals(pointB, pointH, connections.get(index++));

      assertConnectionEquals(pointD, pointF, connections.get(index++));
      assertConnectionEquals(pointD, pointG, connections.get(index++));
      assertConnectionEquals(pointD, pointH, connections.get(index++));

      assertEquals(9, connections.size());
   }

   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   public void testAddCrossClusterVisibilityTwoSomeSquaresWithSomeKeepouts()
   {
      Point2D pointA = new Point2D(0.0, 0.0);
      Point2D pointB = new Point2D(1.0, 0.0);
      Point2D pointC = new Point2D(1.0, 1.0);
      Point2D pointD = new Point2D(0.0, 1.0);

      Point2D pointE = new Point2D(2.0, 0.0);
      Point2D pointF = new Point2D(3.0, 0.0);
      Point2D pointG = new Point2D(3.0, 1.0);
      Point2D pointH = new Point2D(2.0, 1.0);

      // Put a simple square extrusion in the middle of it.
      Cluster clusterOne = new Cluster();

      clusterOne.addNavigableExtrusionInLocal(pointA);
      clusterOne.addNavigableExtrusionInLocal(pointB);
      clusterOne.addNavigableExtrusionInLocal(pointC);
      clusterOne.addNavigableExtrusionInLocal(pointD);

      Cluster clusterTwo = new Cluster();
      clusterTwo.addNavigableExtrusionInLocal(pointE);
      clusterTwo.addNavigableExtrusionInLocal(pointF);
      clusterTwo.addNavigableExtrusionInLocal(pointG);
      clusterTwo.addNavigableExtrusionInLocal(pointH);

      List<Cluster> allClusters = new ArrayList<>();
      allClusters.add(clusterOne);
      allClusters.add(clusterTwo);

      Cluster keepOutClusterOne = new Cluster();
      keepOutClusterOne.addNonNavigableExtrusionInLocal(new Point2D(-0.1, 0.5));
      keepOutClusterOne.addNonNavigableExtrusionInLocal(new Point2D(1.1, 0.5));

      Cluster keepOutClusterTwo = new Cluster();
      keepOutClusterTwo.addNonNavigableExtrusionInLocal(new Point2D(2.5, -0.1));
      keepOutClusterTwo.addNonNavigableExtrusionInLocal(new Point2D(2.5, 1.1));

      allClusters.add(keepOutClusterOne);
      allClusters.add(keepOutClusterTwo);

      int mapId = 77;
      ArrayList<Connection> connections = new ArrayList<>();

      boolean[] sourceNavigability = new boolean[] {true, true, true, true};
      boolean[] targetNavigability = new boolean[] {true, true, true, true};

      VisibilityTools.addCrossClusterVisibility(clusterOne, sourceNavigability, clusterTwo, targetNavigability, allClusters, mapId, connections);

      int index = 0;

      printConnections(connections);
      assertConnectionEquals(pointA, pointE, connections.get(index++));
      assertConnectionEquals(pointB, pointE, connections.get(index++));
      assertConnectionEquals(pointB, pointH, connections.get(index++));
      assertConnectionEquals(pointC, pointE, connections.get(index++));
      assertConnectionEquals(pointC, pointH, connections.get(index++));
      assertConnectionEquals(pointD, pointH, connections.get(index++));

      assertEquals(6, connections.size());
   }

   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   public void testIsPointVisibleForStaticMaps()
   {
      Cluster keepOutClusterOne = new Cluster();
      keepOutClusterOne.addNonNavigableExtrusionInLocal(new Point2D(-0.1, 0.5));
      keepOutClusterOne.addNonNavigableExtrusionInLocal(new Point2D(1.1, 0.5));

      Cluster keepOutClusterTwo = new Cluster();
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
   public void testRemoveConnectionsFromExtrusionsInsideNoGoZones()
   {
      Collection<Connection> connectionsToClean = new ArrayList<>();
      
      ConnectionPoint3D pointA = new ConnectionPoint3D(0.0, 0.0, 0.0, 33);
      ConnectionPoint3D pointB = new ConnectionPoint3D(1.0, 0.0, 0.0, 33);
      ConnectionPoint3D pointC = new ConnectionPoint3D(1.0, 1.0, 0.0, 33);
      ConnectionPoint3D pointD = new ConnectionPoint3D(0.0, 1.0, 0.0, 33);
      
      Connection connectionAB = new Connection(pointA, pointB);
      Connection connectionAC = new Connection(pointA, pointC);
      Connection connectionBD = new Connection(pointB, pointD);
      Connection connectionCA = new Connection(pointC, pointA);
      Connection connectionCB = new Connection(pointC, pointB);
      Connection connectionDC = new Connection(pointD, pointC);
      Connection connectionDA = new Connection(pointD, pointA);
      
      connectionsToClean.add(connectionAB);
      connectionsToClean.add(connectionAC);
      connectionsToClean.add(connectionBD);
      connectionsToClean.add(connectionCA);      
      connectionsToClean.add(connectionCB);      
      connectionsToClean.add(connectionDC);      
      connectionsToClean.add(connectionDA);      
      
      List<Cluster> clusters = new ArrayList<>();
      
      Cluster clusterOne = new Cluster();
      clusterOne.addNonNavigableExtrusionInLocal(new Point2D(-0.1, 0.5));
      clusterOne.addNonNavigableExtrusionInLocal(new Point2D(1.1, 0.5));
      clusters.add(clusterOne);
      
      Cluster clusterTwo = new Cluster();
      clusterTwo.addNonNavigableExtrusionInLocal(new Point2D(0.9, 0.9));
      clusterTwo.addNonNavigableExtrusionInLocal(new Point2D(1.1, 0.9));
      clusterTwo.addNonNavigableExtrusionInLocal(new Point2D(1.1, 1.1));
      clusterTwo.addNonNavigableExtrusionInLocal(new Point2D(0.9, 1.1));
      clusters.add(clusterTwo);
      
      List<Connection> filteredConnections = VisibilityTools.removeConnectionsFromExtrusionsInsideNoGoZones(connectionsToClean, clusters);
      
      assertEquals(3, filteredConnections.size());
      
      int index = 0;
      assertEquals(connectionAB, filteredConnections.get(index++));
      assertEquals(connectionBD, filteredConnections.get(index++));
      assertEquals(connectionDA, filteredConnections.get(index++));
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
