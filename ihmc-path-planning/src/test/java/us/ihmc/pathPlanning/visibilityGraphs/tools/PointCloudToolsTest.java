package us.ihmc.pathPlanning.visibilityGraphs.tools;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class PointCloudToolsTest
{
   private static final double EPSILON = 1e-12;

   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   public void testAddClusterSelfVisibilityOneJustASquare()
   {
      List<Point2DReadOnly> polygonPoints = new ArrayList<>();

      polygonPoints.add(new Point2D(0.0, 0.0));
      polygonPoints.add(new Point2D(1.0, 0.0));
      polygonPoints.add(new Point2D(1.0, 0.2));
      polygonPoints.add(new Point2D(0.0, 0.2));

      double brakeDownThreshold = 0.20001;

      List<Point2DReadOnly> newPolygonPoints = PointCloudTools.addPointsAlongPolygon(polygonPoints, brakeDownThreshold);

      int index = 0;
      assertEquals(12, newPolygonPoints.size());
      assertEpsilonEquals(new Point2D(0.0, 0.0), newPolygonPoints.get(index++));
      assertEpsilonEquals(new Point2D(0.2, 0.0), newPolygonPoints.get(index++));
      assertEpsilonEquals(new Point2D(0.4, 0.0), newPolygonPoints.get(index++));
      assertEpsilonEquals(new Point2D(0.6, 0.0), newPolygonPoints.get(index++));
      assertEpsilonEquals(new Point2D(0.8, 0.0), newPolygonPoints.get(index++));
      assertEpsilonEquals(new Point2D(1.0, 0.0), newPolygonPoints.get(index++));
      assertEpsilonEquals(new Point2D(1.0, 0.2), newPolygonPoints.get(index++));
      assertEpsilonEquals(new Point2D(0.8, 0.2), newPolygonPoints.get(index++));
      assertEpsilonEquals(new Point2D(0.6, 0.2), newPolygonPoints.get(index++));
      assertEpsilonEquals(new Point2D(0.4, 0.2), newPolygonPoints.get(index++));
      assertEpsilonEquals(new Point2D(0.2, 0.2), newPolygonPoints.get(index++));
      assertEpsilonEquals(new Point2D(0.0, 0.2), newPolygonPoints.get(index++));

   }

   private void assertEpsilonEquals(Point2D expectedPoint, Point2DReadOnly actualPoint)
   {
      assertTrue(expectedPoint.epsilonEquals(actualPoint, EPSILON));
   }

   private void printPoints(List<Point2DReadOnly> points)
   {
      for (Point2DReadOnly point : points)
      {
         System.out.println(point);
      }

   }

}
