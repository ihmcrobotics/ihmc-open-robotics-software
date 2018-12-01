package us.ihmc.pathPlanning.visibilityGraphs.tools;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import org.junit.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
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
      List<Point2D> extrusions = ClusterTools.extrudeMultiplePointsAtOutsideCorner(cornerPointToExtrude, previousEdge, nextEdge, extrudeToTheLeft,
                                                                                   numberOfExtrusions, extrusionDistance);

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
      // Simple Square with counterclockwise vertices. Extruding the points to the inside, so should just be a smaller quad.
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
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.1, 0.1), extrudedPolygon.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.8, 0.2), extrudedPolygon.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(1.0, 1.0), extrudedPolygon.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.3, 0.7), extrudedPolygon.get(index++), EPSILON);

      // Simple Square. Extruding the points to the outside, so will round the corners.
      extrudeToTheLeft = false;
      extrudedPolygon = ClusterTools.extrudePolygon(extrudeToTheLeft, pointsToExtrude, extrusionDistances);

      double sqrt2By2 = Math.sqrt(2.0) / 2.0;
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

      // Simple Square with clockwise vertices. Extruding the points to the inside (to the right), so should just be a smaller quad.
      extrudeToTheLeft = false;
      pointsToExtrude = new ArrayList<>();
      pointsToExtrude.add(pointD);
      pointsToExtrude.add(pointC);
      pointsToExtrude.add(pointB);
      pointsToExtrude.add(pointA);

      extrusionDistances = new double[] {0.3, 0.0, 0.2, 0.1};

      extrudedPolygon = ClusterTools.extrudePolygon(extrudeToTheLeft, pointsToExtrude, extrusionDistances);

      assertEquals(4, extrudedPolygon.size());
      index = 0;
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.3, 0.7), extrudedPolygon.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(1.0, 1.0), extrudedPolygon.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.8, 0.2), extrudedPolygon.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.1, 0.1), extrudedPolygon.get(index++), EPSILON);
      
      // Extruding line segment. So will be to the outside, regardless of extrudeToTheLeft. Will round the corners.
      pointsToExtrude = new ArrayList<>();
      pointsToExtrude.add(pointA);
      pointsToExtrude.add(pointB);

      extrudeToTheLeft = false;
      extrusionDistances = new double[] {0.1, 0.1};
      extrudedPolygon = ClusterTools.extrudePolygon(extrudeToTheLeft, pointsToExtrude, extrusionDistances);

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

      double sqrt2By2 = Math.sqrt(2.0) / 2.0;

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

      double sqrt2By2 = Math.sqrt(2.0) / 2.0;

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

      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.1 * sqrt2By2, -0.1 * sqrt2By2), extrudedLine.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.0, -0.1), extrudedLine.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(-0.1 * sqrt2By2, -0.1 * sqrt2By2), extrudedLine.get(index++), EPSILON);
   }

   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   public void testExtrudeSinglePointAtInsideCorner() throws Exception
   {
      Point2D pointA = new Point2D(-1.0, 1.0);
      Point2D pointB = new Point2D(0.0, 0.0);
      Point2D pointC = new Point2D(1.0, 1.0);

      double extrusionDistance = 0.1;
      
      double sqrt2 = Math.sqrt(2.0);
      double sqrt2By2 = Math.sqrt(2.0) / 2.0;

      Point2D expectedAnswer = new Point2D(0.0, 0.1 * sqrt2);
      boolean extrudeToTheLeft = true;
      
      evaluateExtrudeSinglePointAtInsideCorner(pointA, pointB, pointC, extrusionDistance, extrudeToTheLeft, expectedAnswer);
      
      pointA = new Point2D(-100000.0, 1.0);
      pointB = new Point2D(10.0, 1.0);
      pointC = new Point2D(100.0, 1.0);

      extrusionDistance = 0.1;
      expectedAnswer = new Point2D(10.0, 1.1);

      evaluateExtrudeSinglePointAtInsideCorner(pointA, pointB, pointC, extrusionDistance, extrudeToTheLeft, expectedAnswer);
      
      // Check extrude to the right
      pointA = new Point2D(-100000.0, 1.0);
      pointB = new Point2D(10.0, 1.0);
      pointC = new Point2D(100.0, 1.0);

      extrusionDistance = 0.1;
      extrudeToTheLeft = false;

      expectedAnswer = new Point2D(10.0, 0.9);

      evaluateExtrudeSinglePointAtInsideCorner(pointA, pointB, pointC, extrusionDistance, extrudeToTheLeft, expectedAnswer);
      
      // Check lines are shifted by extrusionDistance. Do this by creating the new lines and make sure they are parallel with the old lines.
      pointA = new Point2D(-1.13, 0.37);
      pointB = new Point2D(0.2, 0.11);
      pointC = new Point2D(0.6, 1.0);

      extrusionDistance = 0.1;
      extrudeToTheLeft = true;

      Point2D pointD = extrudeSinglePointAtInsideCorner(pointA, pointB, pointC, extrusionDistance, extrudeToTheLeft);
      
      Line2D lineAB = new Line2D(pointA, pointB);
      Vector2DBasics vectorAB = lineAB.getDirection();
      Vector2D vectorAE = EuclidGeometryTools.perpendicularVector2D(vectorAB);
      
      Line2D lineBC = new Line2D(pointB, pointC);
      Vector2DBasics vectorBC = lineBC.getDirection();
      Vector2D vectorCF = EuclidGeometryTools.perpendicularVector2D(vectorBC);
      
      Point2D pointE = new Point2D();
      pointE.scaleAdd(extrusionDistance, vectorAE, pointA);
      
      Point2D pointF = new Point2D();
      pointF.scaleAdd(extrusionDistance, vectorCF, pointC);
      
      Line2D lineED = new Line2D(pointE, pointD);
      Line2D lineDF = new Line2D(pointD, pointF);
      
      Vector2DBasics vectorED = lineED.getDirection();
      Vector2DBasics vectorDF = lineDF.getDirection();
      
      assertEquals(1.0, vectorAB.dot(vectorED), EPSILON);
      assertEquals(1.0, vectorBC.dot(vectorDF), EPSILON);
   }

   private Point2D evaluateExtrudeSinglePointAtInsideCorner(Point2D pointA, Point2D pointB, Point2D pointC, double extrusionDistance, boolean extrudeToTheLeft, Point2D expectedAnswer)
   {
      Point2D extrudedPoint = extrudeSinglePointAtInsideCorner(pointA, pointB, pointC, extrusionDistance, extrudeToTheLeft);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(expectedAnswer, extrudedPoint, EPSILON);
      
      return extrudedPoint;
   }

   private Point2D extrudeSinglePointAtInsideCorner(Point2D pointA, Point2D pointB, Point2D pointC, double extrusionDistance, boolean extrudeToTheLeft)
   {
      Line2D edgePrev = new Line2D(pointA, pointB);
      Line2D edgeNext = new Line2D(pointB, pointC);

      List<Point2D> extrudedPointList = new ArrayList<>();
      ClusterTools.extrudeSinglePointAtInsideCorner(extrudedPointList, pointB, extrusionDistance, edgePrev, edgeNext, extrudeToTheLeft);
 
      assertEquals(1, extrudedPointList.size());
      Point2D extrudedPoint = extrudedPointList.get(0);
      return extrudedPoint;
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
