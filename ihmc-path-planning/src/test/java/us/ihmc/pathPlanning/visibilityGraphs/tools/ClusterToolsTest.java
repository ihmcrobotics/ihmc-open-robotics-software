package us.ihmc.pathPlanning.visibilityGraphs.tools;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import org.junit.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;

public class ClusterToolsTest
{
   private static final double EPSILON = 1.0e-12;

   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   public void testExtrudeLine() throws Exception
   {
      Point2D endpoint1 = new Point2D(0.0, 0.0);
      Point2D endpoint2 = new Point2D(1.0, 0.0);
      double extrusionDistance = 0.5;

      List<Point2D> extrusions = ClusterTools.extrudeLine(endpoint1, endpoint2, extrusionDistance, 3);

      assertEquals(6, extrusions.size());
      int index = 0;
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.0, -0.5), extrusions.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(-0.5, 0.0), extrusions.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.0, 0.5), extrusions.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(1.0, 0.5), extrusions.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(1.5, 0.0), extrusions.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(1.0, -0.5), extrusions.get(index++), EPSILON);
   }

   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   public void testFilterVerticalPolygonForMultiLineExtrusion() throws Exception
   {
      List<Point3D> rawPoints = new ArrayList<>();
      double expectedObstacleHeight = 1.0;
      rawPoints.add(new Point3D(0.0, 0.0, expectedObstacleHeight));
      rawPoints.add(new Point3D(expectedObstacleHeight, 0.0, expectedObstacleHeight));
      rawPoints.add(new Point3D(0.5, 0.0, 0.0));

      List<Point3D> filteredRawPoints = ClusterTools.filterVerticalPolygonForMultiLineExtrusion(rawPoints, 0.0);

      for (Point3D filteredRawPoint : filteredRawPoints)
      {
         assertEquals(expectedObstacleHeight, filteredRawPoint.getZ(), EPSILON);
      }
   }

   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   public void testExtrudeCorner()
   {
      Point2D previous = new Point2D(1.0, 0.0);
      Point2D cornerPointToExtrude = new Point2D(1.0, 1.0);
      Point2D next = new Point2D(0.0, 1.0);

      Line2D previousEdge = new Line2D(previous, cornerPointToExtrude);
      Line2D nextEdge = new Line2D(cornerPointToExtrude, next);
      boolean extrudeToTheLeft = false;
      int numberOfExtrusions = 3;
      double extrusionDistance = 0.5;
      List<Point2D> extrusions = ClusterTools.extrudeMultiplePointsAtOutsideCorner(cornerPointToExtrude, previousEdge, nextEdge, extrudeToTheLeft, numberOfExtrusions,
                                                            extrusionDistance);

      assertEquals(numberOfExtrusions, extrusions.size());

      Point2D extrusionExpected0 = new Point2D(1.5, 1.0);
      Point2D extrusionExpected1 = new Point2D(1.0, 1.0);
      Vector2D extrusionDirection = new Vector2D(1.0, 1.0);
      extrusionDirection.normalize();
      extrusionExpected1.scaleAdd(extrusionDistance, extrusionDirection, cornerPointToExtrude);
      Point2D extrusionExpected2 = new Point2D(1.0, 1.5);

      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(extrusionExpected0, extrusions.get(0), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(extrusionExpected1, extrusions.get(1), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(extrusionExpected2, extrusions.get(2), EPSILON);
   }
   
   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   public void testExtrudePolygon() throws Exception
   {
      Point2D pointA = new Point2D(0.0, 0.0);
      Point2D pointB = new Point2D(1.0, 0.0);
      Point2D pointC = new Point2D(1.0, 1.0);
      Point2D pointD = new Point2D(0.0, 1.0);
      
      boolean extrudeToTheLeft = true;
      List<Point2DReadOnly> pointsToExtrude = new ArrayList<>();
      pointsToExtrude.add(pointA);
      pointsToExtrude.add(pointB);
      pointsToExtrude.add(pointC);
      pointsToExtrude.add(pointD);
      
      double[] extrusionDistances = new double[] {0.1, 0.2, 0.0, 0.3};
      
      List<Point2D> extrudedPolygon = ClusterTools.extrudePolygon(extrudeToTheLeft, pointsToExtrude, extrusionDistances);
      
      assertEquals(4, extrudedPolygon.size());
      int index = 0;
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.05, 0.05), extrudedPolygon.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.9, 0.1), extrudedPolygon.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(1.0, 1.0), extrudedPolygon.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.15, 0.85), extrudedPolygon.get(index++), EPSILON);

      extrudeToTheLeft = false;
      extrudedPolygon = ClusterTools.extrudePolygon(extrudeToTheLeft, pointsToExtrude, extrusionDistances);

      double sqrt2By2 = Math.sqrt(2.0)/2.0;
      assertEquals(12, extrudedPolygon.size());
      index = 0;
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(-0.1, 0.0), extrudedPolygon.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(-0.1 * sqrt2By2, -0.1 * sqrt2By2), extrudedPolygon.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.0, -0.1), extrudedPolygon.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(1.0, -0.2), extrudedPolygon.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(1.0 + 0.2 * sqrt2By2, -0.2 * sqrt2By2), extrudedPolygon.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(1.2, 0.0), extrudedPolygon.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(1.0, 1.0), extrudedPolygon.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(1.0, 1.0), extrudedPolygon.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(1.0, 1.0), extrudedPolygon.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.0, 1.3), extrudedPolygon.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(-0.3 * sqrt2By2, 1.0 + 0.3 * sqrt2By2), extrudedPolygon.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(-0.3, 1.0), extrudedPolygon.get(index++), EPSILON);

      pointsToExtrude = new ArrayList<>();
      pointsToExtrude.add(pointA);
      pointsToExtrude.add(pointB);
      
      extrudeToTheLeft = false;
      extrusionDistances = new double[] {0.1, 0.1};
      extrudedPolygon = ClusterTools.extrudePolygon(extrudeToTheLeft, pointsToExtrude, extrusionDistances);
      
//      printPoints(extrudedPolygon);
      assertEquals(10, extrudedPolygon.size());
      index = 0;
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.0, -0.1), extrudedPolygon.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(-0.1 * sqrt2By2, -0.1 * sqrt2By2), extrudedPolygon.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(-0.1, 0.0), extrudedPolygon.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(-0.1 * sqrt2By2, 0.1 * sqrt2By2), extrudedPolygon.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.0, 0.1), extrudedPolygon.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(1.0, 0.1), extrudedPolygon.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(1.0 + 0.1 * sqrt2By2, 0.1 * sqrt2By2), extrudedPolygon.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(1.1, 0.0), extrudedPolygon.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(1.0 + 0.1 * sqrt2By2, -0.1 * sqrt2By2), extrudedPolygon.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(1.0, -0.1), extrudedPolygon.get(index++), EPSILON);

   }
   
   
   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   public void testExtrudeTwoPointMultiLine() throws Exception
   {
      Point2D pointA = new Point2D(0.0, 0.0);
      Point2D pointB = new Point2D(1.0, 0.0);
      
      List<Point2DReadOnly> pointsToExtrude = new ArrayList<>();
      pointsToExtrude.add(pointA);
      pointsToExtrude.add(pointB);
      
      double[] extrusionDistances = new double[] {0.1, 0.1};
      int numberOfExtrusionsAtEndpoints = 5;

      List<Point2D> extrudedLine = ClusterTools.extrudeMultiLine(pointsToExtrude, extrusionDistances, numberOfExtrusionsAtEndpoints);
      
//      printPoints(extrudedLine);
      
      double sqrt2By2 = Math.sqrt(2.0)/2.0;

      assertEquals(10, extrudedLine.size());
      int index = 0;
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.0, -0.1), extrudedLine.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(-0.1 * sqrt2By2, -0.1 * sqrt2By2), extrudedLine.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(-0.1, 0.0), extrudedLine.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(-0.1 * sqrt2By2, 0.1 * sqrt2By2), extrudedLine.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.0, 0.1), extrudedLine.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(1.0, 0.1), extrudedLine.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(1.0 + 0.1 * sqrt2By2, 0.1 * sqrt2By2), extrudedLine.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(1.1, 0.0), extrudedLine.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(1.0 + 0.1 * sqrt2By2, -0.1 * sqrt2By2), extrudedLine.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(1.0, -0.1), extrudedLine.get(index++), EPSILON);
   }
   
   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   public void testExtrudeMultiLine() throws Exception
   {
      Point2D pointA = new Point2D(-1.0, 1.0);
      Point2D pointB = new Point2D(0.0, 0.0);
      Point2D pointC = new Point2D(1.0, 1.0);
      
      List<Point2DReadOnly> pointsToExtrude = new ArrayList<>();
      pointsToExtrude.add(pointA);
      pointsToExtrude.add(pointB);
      pointsToExtrude.add(pointC);
      
      double[] extrusionDistances = new double[] {0.1, 0.1, 0.1};
      int numberOfExtrusionsAtEndpoints = 5;

      List<Point2D> extrudedLine = ClusterTools.extrudeMultiLine(pointsToExtrude, extrusionDistances, numberOfExtrusionsAtEndpoints);
            
      double sqrt2By2 = Math.sqrt(2.0)/2.0;
      
      assertEquals(14, extrudedLine.size());
      int index = 0;
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(-1.0 - 0.1 * sqrt2By2, 1.0 - 0.1 * sqrt2By2), extrudedLine.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(-1.0 - 0.1, 1.0), extrudedLine.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(-1.0 - 0.1 * sqrt2By2, 1.0 + 0.1 * sqrt2By2), extrudedLine.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(-1.0, 1.0 + 0.1), extrudedLine.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(-1.0 + 0.1 * sqrt2By2, 1.0 + 0.1 * sqrt2By2), extrudedLine.get(index++), EPSILON);
      
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.0, 0.1 * Math.sqrt(2.0)), extrudedLine.get(index++), EPSILON);
      
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(1.0 - 0.1 * sqrt2By2, 1.0 + 0.1 * sqrt2By2), extrudedLine.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(1.0, 1.0 + 0.1), extrudedLine.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(1.0 + 0.1 * sqrt2By2, 1.0 + 0.1 * sqrt2By2), extrudedLine.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(1.0 + 0.1, 1.0), extrudedLine.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(1.0 + 0.1 * sqrt2By2, 1.0 - 0.1 * sqrt2By2), extrudedLine.get(index++), EPSILON);
      
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.1 * sqrt2By2, - 0.1 * sqrt2By2), extrudedLine.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.0, - 0.1), extrudedLine.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(-0.1 * sqrt2By2, - 0.1 * sqrt2By2), extrudedLine.get(index++), EPSILON);
   }
   
   private void printPoints(Collection<Point2D> points)
   {
      //      System.out.print("{");

      for (Point2D point : points)
      {
         System.out.println(point);

         //         System.out.print("{" + connection.getSourcePoint().getX() + ", " + connection.getSourcePoint().getY() + "}" + ",");
      }
      //      System.out.println("}");

   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(ClusterTools.class, ClusterToolsTest.class);

   }

}
