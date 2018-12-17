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
   public void testIsPointVisibleForStaticMapsClosedPolygonVsOpenMultiLine()
   {
      Cluster keepOutClusterPolygon = new Cluster();
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
