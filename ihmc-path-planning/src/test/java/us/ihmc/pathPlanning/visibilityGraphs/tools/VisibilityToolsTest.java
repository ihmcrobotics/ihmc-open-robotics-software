package us.ihmc.pathPlanning.visibilityGraphs.tools;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.axisAngle.AxisAngle;
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
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.Connection;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.ConnectionPoint3D;
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

   @Test
   public void testDistanceBetweenTwoLineSegment2Ds()
   {
      Point2D closestPointOnLineSegment1 = new Point2D();
      Point2D closestPointOnLineSegment2 = new Point2D();

      Vector2D lineSegmentDirection1 = new Vector2D();
      Vector2D lineSegmentDirection2 = new Vector2D();

      Random random = new Random(11762L);


      // Parallel case, expecting expectedPointOnLineSegment1 =
      // lineSegmentStart1
      for (int i = 0; i < iters; i++)
      {
         Point2D lineSegmentStart1 = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D lineSegmentEnd1 = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         lineSegmentDirection1.sub(lineSegmentEnd1, lineSegmentStart1);
         lineSegmentDirection1.normalize();

         // expectedPointOnLineSegment1 = lineSegmentStart1
         closestPointOnLineSegment1.set(lineSegmentStart1);

         // Create the closest point of line segment 2
         Vector2D orthogonalToLineSegment1 = nextOrthogonalVector2D(random, lineSegmentDirection1, true);
         double expectedMinimumDistance = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         closestPointOnLineSegment2.scaleAdd(expectedMinimumDistance, orthogonalToLineSegment1, closestPointOnLineSegment1);

         // Set the lineSegmentDirection2 = lineSegmentDirection1
         lineSegmentDirection2.set(lineSegmentDirection1);

         // Set the end points of the line segment 2 around the expected
         // closest point.
         Point2D lineSegmentStart2 = new Point2D();
         Point2D lineSegmentEnd2 = new Point2D();
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), lineSegmentDirection2, closestPointOnLineSegment2);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineSegmentDirection2, closestPointOnLineSegment2);

         double actualMinimumDistance = VisibilityTools.distanceBetweenTwoLineSegment2Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2,
                                                                                             lineSegmentEnd2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);

         // Set the end points of the line segment 2 before the expected
         // closest point, so we have expectedClosestPointOnLineSegment2 =
         // lineSegmentEnd2
         double shiftStartFromExpected = EuclidCoreRandomTools.nextDouble(random, -20.0, -10.0);
         double shiftEndFromExpected = EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0);
         lineSegmentStart2.scaleAdd(shiftStartFromExpected, lineSegmentDirection2, closestPointOnLineSegment2);
         lineSegmentEnd2.scaleAdd(shiftEndFromExpected, lineSegmentDirection2, closestPointOnLineSegment2);
         closestPointOnLineSegment2.set(lineSegmentEnd2);
         expectedMinimumDistance = closestPointOnLineSegment1.distance(closestPointOnLineSegment2);

         actualMinimumDistance = VisibilityTools.distanceBetweenTwoLineSegment2Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);

         actualMinimumDistance = VisibilityTools.distanceBetweenTwoLineSegment2Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
      }

      // Case: on closest point on lineSegment1 outside end points.
      for (int i = 0; i < iters; i++)
      {
         Point2D lineSegmentStart1 = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D lineSegmentEnd1 = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         lineSegmentDirection1.sub(lineSegmentEnd1, lineSegmentStart1);
         lineSegmentDirection1.normalize();

         // Put the first closest to the start of line segment 1
         closestPointOnLineSegment1.set(lineSegmentStart1);

         // Create the closest point of line segment 2 such that it reaches
         // out of line segment 1
         Vector2D oppositeOflineSegmentDirection1 = new Vector2D();
         oppositeOflineSegmentDirection1.setAndNegate(lineSegmentDirection1);
         Vector2D orthogonalToLineSegment1 = nextOrthogonalVector2D(random, lineSegmentDirection1, true);
         Vector2D shiftVector = new Vector2D();
         shiftVector.interpolate(orthogonalToLineSegment1, oppositeOflineSegmentDirection1, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         closestPointOnLineSegment2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), shiftVector, closestPointOnLineSegment1);

         // Set the line direction 2 to orthogonal to the shift vector
         lineSegmentDirection2 = nextOrthogonalVector2D(random, shiftVector, true);

         // Set the end points of the line segment 2 around the expected
         // closest point.
         Point2D lineSegmentStart2 = new Point2D();
         Point2D lineSegmentEnd2 = new Point2D();
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), lineSegmentDirection2, closestPointOnLineSegment2);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineSegmentDirection2, closestPointOnLineSegment2);
         double expectedMinimumDistance = closestPointOnLineSegment1.distance(closestPointOnLineSegment2);

         double actualMinimumDistance = VisibilityTools.distanceBetweenTwoLineSegment2Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2,
                                                                                             lineSegmentEnd2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
         actualMinimumDistance = VisibilityTools.distanceBetweenTwoLineSegment2Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
         actualMinimumDistance = VisibilityTools.distanceBetweenTwoLineSegment2Ds(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
         actualMinimumDistance = VisibilityTools.distanceBetweenTwoLineSegment2Ds(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
      }

      // Edge case: both closest points are outside bounds of each line
      // segment
      for (int i = 0; i < iters; i++)
      {
         Point2D lineSegmentStart1 = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D lineSegmentEnd1 = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         lineSegmentDirection1.sub(lineSegmentEnd1, lineSegmentStart1);
         lineSegmentDirection1.normalize();

         // Put the first closest to the start of line segment 1
         closestPointOnLineSegment1.set(lineSegmentStart1);

         // Create the closest point of line segment 2 such that it reaches
         // out of line segment 1
         Vector2D oppositeOflineSegmentDirection1 = new Vector2D();
         oppositeOflineSegmentDirection1.setAndNegate(lineSegmentDirection1);
         Vector2D orthogonalToLineSegment1 = nextOrthogonalVector2D(random, lineSegmentDirection1, true);
         Vector2D shiftVector = new Vector2D();
         double alpha = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         shiftVector.interpolate(orthogonalToLineSegment1, oppositeOflineSegmentDirection1, alpha);
         closestPointOnLineSegment2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), shiftVector, closestPointOnLineSegment1);

         // set the start of the second line segment to the expected closest
         // point
         Point2D lineSegmentStart2 = new Point2D(closestPointOnLineSegment2);

         // Set the line direction 2 to point somewhat in the same direction
         // as the shift vector
         Vector2D orthogonalToShiftVector = nextOrthogonalVector2D(random, shiftVector, true);
         lineSegmentDirection2.interpolate(shiftVector, orthogonalToShiftVector, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

         // Set the end points of the line segment 2 around the expected
         // closest point.
         Point2D lineSegmentEnd2 = new Point2D();
         double alpha2 = EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0);
         lineSegmentEnd2.scaleAdd(alpha2, lineSegmentDirection2, closestPointOnLineSegment2);

         double expectedMinimumDistance = closestPointOnLineSegment1.distance(closestPointOnLineSegment2);
         double actualMinimumDistance = VisibilityTools.distanceBetweenTwoLineSegment2Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2,
                                                                                             lineSegmentEnd2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
         actualMinimumDistance = VisibilityTools.distanceBetweenTwoLineSegment2Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
         actualMinimumDistance = VisibilityTools.distanceBetweenTwoLineSegment2Ds(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
         actualMinimumDistance = VisibilityTools.distanceBetweenTwoLineSegment2Ds(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
      }
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

   /**
    * Generates a random vector that is perpendicular to {@code vectorToBeOrthogonalTo}.
    *
    * @param random the random generator to use.
    * @param vectorToBeOrthogonalTo the vector to be orthogonal to. Not modified.
    * @param normalize whether to normalize the generated vector or not.
    * @return the random vector.
    */
   public static Vector2D nextOrthogonalVector2D(Random random, Vector2DReadOnly vectorToBeOrthogonalTo, boolean normalize)
   {
      Vector2D v1 = new Vector2D(vectorToBeOrthogonalTo.getY(), -vectorToBeOrthogonalTo.getX());

      Vector2D randomPerpendicular = new Vector2D();
      double a = nextDouble(random, 1.0);
      randomPerpendicular.scaleAdd(a, v1, randomPerpendicular);

      if (normalize)
         randomPerpendicular.normalize();

      return randomPerpendicular;
   }

   public static double nextDouble(Random random, double minMaxValue)
   {
      return nextDouble(random, -minMaxValue, minMaxValue);
   }

   public static double nextDouble(Random random, double minValue, double maxValue)
   {
      if (minValue > maxValue)
         throw new RuntimeException("Min is greater than max: min = " + minValue + ", max = " + maxValue);

      return minValue + random.nextDouble() * (maxValue - minValue);
   }


   public static void main(String[] args) throws IOException
   {
      MutationTestFacilitator.facilitateMutationTestForClass(VisibilityTools.class, VisibilityToolsTest.class);
   }

}
