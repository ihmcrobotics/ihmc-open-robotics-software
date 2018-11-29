package us.ihmc.pathPlanning.visibilityGraphs.tools;

import static org.junit.Assert.assertEquals;

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


//      printConnections(connections);

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

      printConnections(connections);

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
