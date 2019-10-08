package us.ihmc.pathPlanning.visibilityGraphs.tools;

import static us.ihmc.robotics.Assert.*;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.ObstacleExtrusionDistanceCalculator;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.ObstacleRegionFilter;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionFilter;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegion;

public class ClusterToolsTest
{
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testExtrudeLine() throws Exception
   {
      Point2D endpoint1 = new Point2D(0.0, 0.0);
      Point2D endpoint2 = new Point2D(1.0, 0.0);
      double extrusionDistance = 0.5;

      List<Point2DReadOnly> points = new ArrayList<Point2DReadOnly>();
      points.add(endpoint1);
      points.add(endpoint2);

      double[] extrusionDistances = new double[] {extrusionDistance, extrusionDistance};
      List<Point2D> extrusions = ClusterTools.extrudeMultiLine(points, extrusionDistances, 3);

      assertEquals(6, extrusions.size());
      int index = 0;
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.0, -0.5), extrusions.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(-0.5, 0.0), extrusions.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.0, 0.5), extrusions.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(1.0, 0.5), extrusions.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(1.5, 0.0), extrusions.get(index++), EPSILON);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(1.0, -0.5), extrusions.get(index++), EPSILON);
   }

   @Test
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

   @Test
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

   @Test
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

   @Test
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

   @Test
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

   @Test
   public void testExtrudeSinglePointAtInsideCorner() throws Exception
   {
      Point2D pointA = new Point2D(-1.0, 1.0);
      Point2D pointB = new Point2D(0.0, 0.0);
      Point2D pointC = new Point2D(1.0, 1.0);

      double extrusionDistance = 0.1;

      double sqrt2 = Math.sqrt(2.0);

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

   private Point2D evaluateExtrudeSinglePointAtInsideCorner(Point2D pointA, Point2D pointB, Point2D pointC, double extrusionDistance, boolean extrudeToTheLeft,
                                                            Point2D expectedAnswer)
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
      extrudedPointList.add(ClusterTools.extrudeSinglePointAtInsideCorner(pointB, edgePrev, edgeNext, extrudeToTheLeft, extrusionDistance));

      assertEquals(1, extrudedPointList.size());
      Point2D extrudedPoint = extrudedPointList.get(0);
      return extrudedPoint;
   }

   @Test
   public void testSquareOnTopOfSquare() throws Exception
   {
      double tiltedPointHeight = 0.1;
      double arbitraryTranslationX = 3.0;
      double arbitraryTranslationY = -4.0;

      Point3D pointAInWorld = new Point3D(0.0, 0.0, 0.0);
      Point3D pointBInWorld = new Point3D(0.0, 1.0, 0.0);
      Point3D pointCInWorld = new Point3D(1.0, 1.0, tiltedPointHeight);
      Point3D pointDInWorld = new Point3D(1.0, 0.0, tiltedPointHeight);

      Vector3D zAxisForTransform = new Vector3D(-tiltedPointHeight, 0.0, 1.0);
      zAxisForTransform.normalize();

      RigidBodyTransform transformToWorldOops = new RigidBodyTransform();
      transformToWorldOops.setTranslation(arbitraryTranslationX, arbitraryTranslationY, 0.0);

      RigidBodyTransform transformToWorld0 = createTransformFromPointAndZAxis(new Point3D(), zAxisForTransform);
      transformToWorld0.multiply(transformToWorldOops);

      RigidBodyTransform transformToLocal0 = new RigidBodyTransform(transformToWorld0);
      transformToLocal0.invert();

      Point3D pointAInLocal3D = createAndTransformPoint(pointAInWorld, transformToLocal0);
      Point3D pointBInLocal3D = createAndTransformPoint(pointBInWorld, transformToLocal0);
      Point3D pointCInLocal3D = createAndTransformPoint(pointCInWorld, transformToLocal0);
      Point3D pointDInLocal3D = createAndTransformPoint(pointDInWorld, transformToLocal0);

      assertEquals(0.0, pointAInLocal3D.getZ(), EPSILON);
      assertEquals(0.0, pointBInLocal3D.getZ(), EPSILON);
      assertEquals(0.0, pointCInLocal3D.getZ(), EPSILON);
      assertEquals(0.0, pointDInLocal3D.getZ(), EPSILON);

      Point2D pointAInLocal = new Point2D(pointAInLocal3D);
      Point2D pointBInLocal = new Point2D(pointBInLocal3D);
      Point2D pointCInLocal = new Point2D(pointCInLocal3D);
      Point2D pointDInLocal = new Point2D(pointDInLocal3D);

      Vertex2DSupplier vertices0 = Vertex2DSupplier.asVertex2DSupplier(pointAInLocal, pointBInLocal, pointCInLocal, pointDInLocal);

      ConvexPolygon2D convexPolygon0 = new ConvexPolygon2D(vertices0);
      PlanarRegion homeRegion = new PlanarRegion(transformToWorld0, convexPolygon0);

      Point2D pointEInLocal = new Point2D(0.3, 0.3);
      Point2D pointFInLocal = new Point2D(0.3, 0.7);
      Point2D pointGInLocal = new Point2D(0.7, 0.7);
      Point2D pointHInLocal = new Point2D(0.7, 0.3);

      RigidBodyTransform transformToWorld1 = new RigidBodyTransform();
      transformToWorld1.setTranslation(0.0, 0.0, 0.5);

      Vertex2DSupplier vertices1 = Vertex2DSupplier.asVertex2DSupplier(pointEInLocal, pointFInLocal, pointGInLocal, pointHInLocal);
      ConvexPolygon2D convexPolygon1 = new ConvexPolygon2D(vertices1);

      PlanarRegion obstacleRegion = new PlanarRegion(transformToWorld1, convexPolygon1);

      List<PlanarRegion> obstacleRegions = new ArrayList<>();
      obstacleRegions.add(obstacleRegion);

      double orthogonalAngle = 0.8;
      ObstacleExtrusionDistanceCalculator extrusionDistanceCalculator = new ObstacleExtrusionDistanceCalculator()
      {
         @Override
         public double computeExtrusionDistance(Point2DReadOnly pointToExtrude, double obstacleHeight)
         {
            return 0.1;
         }
      };

      List<Cluster> obstacleClusters = ClusterTools.createObstacleClusters(homeRegion, obstacleRegions, orthogonalAngle, extrusionDistanceCalculator);

      assertEquals(1, obstacleClusters.size());
      Cluster obstacleCluster = obstacleClusters.get(0);
      List<Point3DReadOnly> navigableExtrusionsInWorld = obstacleCluster.getNavigableExtrusionsInWorld();

      assertEquals(12, navigableExtrusionsInWorld.size());

      //      printPoints3D(navigableExtrusionsInWorld);
      double sqrt2By2 = Math.sqrt(2.0) / 2.0;

      Point2D pointE0InWorld = new Point2D(0.3, 0.2);
      Point2D pointE1InWorld = new Point2D(0.3 - sqrt2By2 * 0.1, 0.3 - sqrt2By2 * 0.1);
      Point2D pointE2InWorld = new Point2D(0.2, 0.3);

      Point2D pointF0InWorld = new Point2D(0.2, 0.7);
      Point2D pointF1InWorld = new Point2D(0.3 - sqrt2By2 * 0.1, 0.7 + sqrt2By2 * 0.1);
      Point2D pointF2InWorld = new Point2D(0.3, 0.8);

      Point2D pointG0InWorld = new Point2D(0.7, 0.8);
      Point2D pointG1InWorld = new Point2D(0.7 + sqrt2By2 * 0.1, 0.7 + sqrt2By2 * 0.1);
      Point2D pointG2InWorld = new Point2D(0.8, 0.7);

      Point2D pointH0InWorld = new Point2D(0.8, 0.3);
      Point2D pointH1InWorld = new Point2D(0.7 + sqrt2By2 * 0.1, 0.3 - sqrt2By2 * 0.1);
      Point2D pointH2InWorld = new Point2D(0.7, 0.2);

      assertTrue(listContainsAllXYMatch(navigableExtrusionsInWorld, pointE0InWorld, pointE1InWorld, pointE2InWorld));
      assertTrue(listContainsAllXYMatch(navigableExtrusionsInWorld, pointF0InWorld, pointF1InWorld, pointF2InWorld));
      assertTrue(listContainsAllXYMatch(navigableExtrusionsInWorld, pointG0InWorld, pointG1InWorld, pointG2InWorld));
      assertTrue(listContainsAllXYMatch(navigableExtrusionsInWorld, pointH0InWorld, pointH1InWorld, pointH2InWorld));

      // Make sure navigableExtrusionsInWorld lie on the PlanarRegion.
      RigidBodyTransform transformToPlanarRegion = new RigidBodyTransform();
      homeRegion.getTransformToWorld(transformToPlanarRegion);
      transformToPlanarRegion.invert();

      for (Point3DReadOnly point3D : navigableExtrusionsInWorld)
      {
         Point3D pointToTest = new Point3D(point3D);
         transformToPlanarRegion.transform(pointToTest);

         assertEquals(0.0, pointToTest.getZ(), EPSILON);
      }
   }

   @Test
   public void testTwoSquaresOneObstacle() throws Exception
   {
      Point2D[] region0_1Points = new Point2D[] {new Point2D(-3.0, 3.0), new Point2D(3.0, 3.0), new Point2D(3.0, -3.0), new Point2D(-3.0, -3.0)};
      Vector3D normal0_1 = new Vector3D(0.0, 0.0, 1.0);
      normal0_1.normalize();
      RigidBodyTransform transform0_1 = createTransformFromPointAndZAxis(new Point3D(), normal0_1);
      PlanarRegion region0_1 = createPlanarRegion(transform0_1, region0_1Points);

      double distanceBetweenBottomSquares = 0.25;

      Point2D[] region1_1Points = new Point2D[] {new Point2D(3.0 + distanceBetweenBottomSquares, 3.0), new Point2D(7.0, 3.0), new Point2D(7.0, -3.0),
            new Point2D(3.0 + distanceBetweenBottomSquares, -3.0)};
      Vector3D normal1_1 = new Vector3D(0.0, 0.0, 1.0);
      normal1_1.normalize();
      RigidBodyTransform transform1_1 = createTransformFromPointAndZAxis(new Point3D(), normal1_1);
      PlanarRegion region1_1 = createPlanarRegion(transform1_1, region1_1Points);

      Point2D[] region2_1Points = new Point2D[] {new Point2D(-0.5, 2.0), new Point2D(0.5, 2.0), new Point2D(0.5, -2.0), new Point2D(-0.5, -2.0)};
      Vector3D normal2_1 = new Vector3D(0.0, 0.0, 1.0);
      normal2_1.normalize();
      RigidBodyTransform transform2_1 = createTransformFromPointAndZAxis(new Point3D(3.0 + distanceBetweenBottomSquares / 2.0, 0.0, 1.0), normal2_1);
      PlanarRegion region2_1 = createPlanarRegion(transform2_1, region2_1Points);

      List<PlanarRegion> obstacleRegions = new ArrayList<>();
      obstacleRegions.add(region0_1);
      obstacleRegions.add(region1_1);
      PlanarRegion homeRegion = region2_1;

      List<Cluster> obstacleClusters = createObstacleClustersForTests(obstacleRegions, homeRegion);
      assertEquals(0, obstacleClusters.size());

      obstacleRegions.clear();
      obstacleRegions.add(region1_1);
      homeRegion = region0_1;

      obstacleClusters = createObstacleClustersForTests(obstacleRegions, homeRegion);
      assertEquals(0, obstacleClusters.size());

      obstacleRegions.clear();
      obstacleRegions.add(region1_1);
      obstacleRegions.add(region2_1);
      homeRegion = region0_1;

      obstacleClusters = createObstacleClustersForTests(obstacleRegions, homeRegion);
      assertEquals(1, obstacleClusters.size());
      Cluster cluster = obstacleClusters.get(0);
      List<Point2DReadOnly> nonNavigableExtrusionsInLocal = cluster.getNonNavigableExtrusionsInLocal();
      assertEquals(12, nonNavigableExtrusionsInLocal.size());

      obstacleRegions.clear();
      obstacleRegions.add(region0_1);
      obstacleRegions.add(region2_1);
      homeRegion = region1_1;

      obstacleClusters = createObstacleClustersForTests(obstacleRegions, homeRegion);
      assertEquals(1, obstacleClusters.size());
      cluster = obstacleClusters.get(0);
      nonNavigableExtrusionsInLocal = cluster.getNonNavigableExtrusionsInLocal();
      assertEquals(12, nonNavigableExtrusionsInLocal.size());
   }

   @Test
   public void testCreateObstaclesVerticalSegmentOverRotatedBase()
   {
      Point2D pointA = new Point2D(0.0, -0.5);
      Point2D pointB = new Point2D(0.0, 0.5);
      Point2D pointC = new Point2D(0.5, 0.5);
      Point2D pointD = new Point2D(0.5, -0.5);

      Point2D pointE = new Point2D(-10.0, -10.0);
      Point2D pointF = new Point2D(-10.0, 10.0);
      Point2D pointG = new Point2D(10.0, 10.0);
      Point2D pointH = new Point2D(10.0, -10.0);

      ConvexPolygon2D polygon0_0 = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(pointA, pointB, pointC, pointD));
      ConvexPolygon2D polygon1_0 = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(pointE, pointF, pointG, pointH));

      RigidBodyTransform transform0 = new RigidBodyTransform();
      transform0.setRotationEuler(0.0, Math.PI / 2.0, 0.0);
      transform0.setTranslation(0.0, 0.0, 1.0);
      PlanarRegion planarRegion0 = new PlanarRegion(transform0, polygon0_0);

      ObstacleExtrusionDistanceCalculator extrusionDistanceCalculator = new ObstacleExtrusionDistanceCalculator()
      {
         @Override
         public double computeExtrusionDistance(Point2DReadOnly pointToExtrude, double obstacleHeight)
         {
            return 0.01;
         }
      };
      double orthogonalAngle = 0.5;

      double baseRotationAngle = 0.0;
      RigidBodyTransform transform1 = new RigidBodyTransform();
      transform1.setRotationEuler(0.0, baseRotationAngle, 0.0);
      PlanarRegion planarRegion1 = new PlanarRegion(transform1, polygon1_0);

      List<PlanarRegion> obstacleRegions = new ArrayList<>();
      obstacleRegions.add(planarRegion0);
      List<Cluster> obstacleClustersOn1 = ClusterTools.createObstacleClusters(planarRegion1, obstacleRegions, orthogonalAngle, extrusionDistanceCalculator);

      assertEquals(1, obstacleClustersOn1.size());
      Cluster obstacleCluster = obstacleClustersOn1.get(0);

      List<Point3DReadOnly> obstacleExtrusionsInWorld = obstacleCluster.getNavigableExtrusionsInWorld();
      assertEquals(10, obstacleExtrusionsInWorld.size());

      assertTrue(listContains(obstacleExtrusionsInWorld, new Point3D(0.0, -0.51, 0.0)));
      assertTrue(listContains(obstacleExtrusionsInWorld, new Point3D(0.0, 0.51, 0.0)));
      assertTrue(listContainsXYMatch(obstacleExtrusionsInWorld, new Point2D(0.01, 0.5)));
      assertTrue(listContainsXYMatch(obstacleExtrusionsInWorld, new Point2D(0.01, -0.5)));
      assertTrue(listContainsXYMatch(obstacleExtrusionsInWorld, new Point2D(-0.01, 0.5)));
      assertTrue(listContainsXYMatch(obstacleExtrusionsInWorld, new Point2D(-0.01, -0.5)));

      baseRotationAngle = 0.2;
      transform1 = new RigidBodyTransform();
      transform1.setRotationEuler(0.0, baseRotationAngle, 0.0);
      planarRegion1 = new PlanarRegion(transform1, polygon1_0);

      obstacleRegions = new ArrayList<>();
      obstacleRegions.add(planarRegion0);
      obstacleClustersOn1 = ClusterTools.createObstacleClusters(planarRegion1, obstacleRegions, orthogonalAngle, extrusionDistanceCalculator);

      assertEquals(1, obstacleClustersOn1.size());
      obstacleCluster = obstacleClustersOn1.get(0);

      obstacleExtrusionsInWorld = obstacleCluster.getNavigableExtrusionsInWorld();
      assertEquals(10, obstacleExtrusionsInWorld.size());

      //      printPoints3D(obstacleExtrusionsInWorld);

      assertTrue(listContains(obstacleExtrusionsInWorld, new Point3D(0.0, -0.51, 0.0)));
      assertTrue(listContains(obstacleExtrusionsInWorld, new Point3D(0.0, 0.51, 0.0)));
      assertTrue(listContainsXYMatch(obstacleExtrusionsInWorld, new Point2D(0.01, 0.5)));
      assertTrue(listContainsXYMatch(obstacleExtrusionsInWorld, new Point2D(0.01, -0.5)));
      assertTrue(listContainsXYMatch(obstacleExtrusionsInWorld, new Point2D(-0.01, 0.5)));
      assertTrue(listContainsXYMatch(obstacleExtrusionsInWorld, new Point2D(-0.01, -0.5)));

   }

   @Test
   public void testFilterPointsWithSameXYCoordinatesKeepingHighest()
   {
      List<Point3D> pointsToFilter = new ArrayList<>();
      double thresholdSquared = 0.1 * 0.1;

      Point3D pointA = new Point3D(0.0, 0.0, 0.7);
      Point3D pointB = new Point3D(0.001, 0.0, 0.3);
      Point3D pointC = new Point3D(0.002, 0.0, -0.9);
      Point3D pointD = new Point3D(0.01, 0.0, 0.9);
      Point3D pointE = new Point3D(0.3, 0.0, 0.55);
      Point3D pointF = new Point3D(0.35, 0.0, 0.75);
      Point3D pointG = new Point3D(0.451, 0.0, 0.43);
      Point3D pointH = new Point3D(0.54, 0.0, 0.2);

      pointsToFilter.add(pointA);
      pointsToFilter.add(pointB);

      List<Point3D> filteredPoints = ClusterTools.filterPointsWithSameXYCoordinatesKeepingHighest(pointsToFilter, thresholdSquared);
      assertEquals(1, filteredPoints.size());
      assertTrue(listContains(filteredPoints, pointA));

      pointsToFilter.clear();
      pointsToFilter.add(pointB);
      pointsToFilter.add(pointA);

      filteredPoints = ClusterTools.filterPointsWithSameXYCoordinatesKeepingHighest(pointsToFilter, thresholdSquared);
      assertEquals(1, filteredPoints.size());
      assertTrue(listContains(filteredPoints, pointA));

      pointsToFilter.clear();
      pointsToFilter.add(pointB);
      pointsToFilter.add(pointC);
      pointsToFilter.add(pointA);
      filteredPoints = ClusterTools.filterPointsWithSameXYCoordinatesKeepingHighest(pointsToFilter, thresholdSquared);
      assertEquals(1, filteredPoints.size());
      assertTrue(listContains(filteredPoints, pointA));

      pointsToFilter.clear();
      pointsToFilter.add(pointA);
      pointsToFilter.add(pointB);
      pointsToFilter.add(pointC);
      pointsToFilter.add(pointD);
      pointsToFilter.add(pointE);
      pointsToFilter.add(pointF);
      pointsToFilter.add(pointG);
      pointsToFilter.add(pointH);

      filteredPoints = ClusterTools.filterPointsWithSameXYCoordinatesKeepingHighest(pointsToFilter, thresholdSquared);
      assertEquals(3, filteredPoints.size());
      assertTrue(listContains(filteredPoints, pointD));
      assertTrue(listContains(filteredPoints, pointF));
      assertTrue(listContains(filteredPoints, pointG));

      pointsToFilter.clear();
      for (int i = 0; i < 101; i++)
      {
         pointsToFilter.add(new Point3D(0.03 * i, 0.0, 0.03 * i));
      }

      filteredPoints = ClusterTools.filterPointsWithSameXYCoordinatesKeepingHighest(pointsToFilter, thresholdSquared);
      assertEquals(25, filteredPoints.size());
      assertTrue(listContains(filteredPoints, new Point3D(3.0, 0.0, 3.0)));

      pointsToFilter.clear();
      for (int i = 0; i < 101; i++)
      {
         pointsToFilter.add(new Point3D(0.03 * i, 0.0, 3.0 - 0.03 * i));
      }

      filteredPoints = ClusterTools.filterPointsWithSameXYCoordinatesKeepingHighest(pointsToFilter, thresholdSquared);
      assertEquals(26, filteredPoints.size());
      assertTrue(listContains(filteredPoints, new Point3D(0.0, 0.0, 3.0)));
   }

   @Test
   public void testVerticalObstacleOne()
   {
      Point2D[] region0_1Points = new Point2D[] {new Point2D(-1.0, 1.0), new Point2D(1.0, 1.0), new Point2D(1.0, -1.0), new Point2D(-1.0, -1.0)};
      Vector3D normal0_1 = new Vector3D(0.0, 0.0, 1.0);
      normal0_1.normalize();
      RigidBodyTransform transform0_1 = new RigidBodyTransform();
      PlanarRegion flatGroundRegion = createPlanarRegion(transform0_1, region0_1Points);

      Point2D[] region1_1Points = new Point2D[] {new Point2D(0.0, 0.0), new Point2D(0.0, 0.2), new Point2D(0.1, 0.2), new Point2D(0.1, 0.0)};
      Point2D[] region1_2Points = new Point2D[] {new Point2D(0.1, 0.0), new Point2D(0.1, 0.4), new Point2D(0.2, 0.4), new Point2D(0.2, 0.0)};
      Point2D[] concaveHull = new Point2D[] {new Point2D(0.0, 0.0), new Point2D(0.0, 0.2), new Point2D(0.1, 0.2), new Point2D(0.1, 0.4), new Point2D(0.2, 0.4),
            new Point2D(0.2, 0.0)};
      Vector3D normal1_1 = new Vector3D(0.0, -1.0, 0.0);
      normal1_1.normalize();
      RigidBodyTransform transform1_1 = createTransformFromPointAndZAxis(new Point3D(0.0, 0.0, 0.014), normal1_1);
      PlanarRegion verticalObstacleRegion = createPlanarRegionFromSeveralPolygons(concaveHull, transform1_1, region1_1Points, region1_2Points);

      List<PlanarRegion> obstacleRegions = new ArrayList<>();
      obstacleRegions.add(verticalObstacleRegion);
      PlanarRegion homeRegion = flatGroundRegion;

      DefaultVisibilityGraphParameters parameters = new DefaultVisibilityGraphParameters();

      double orthogonalAngle = parameters.getRegionOrthogonalAngle();
      ObstacleExtrusionDistanceCalculator extrusionDistanceCalculator = new ObstacleExtrusionDistanceCalculator()
      {
         @Override
         public double computeExtrusionDistance(Point2DReadOnly pointToExtrude, double obstacleHeight)
         {
            if (obstacleHeight < (0.014 + 0.2 - 0.001))
               return 0.0;
            if (obstacleHeight < (0.014 + 0.4 - 0.001))
               return 0.2;
            return 0.4;
         }
      };

      List<Cluster> obstacleClusters = ClusterTools.createObstacleClusters(homeRegion, obstacleRegions, orthogonalAngle, extrusionDistanceCalculator);
      assertEquals(1, obstacleClusters.size());

      Cluster cluster = obstacleClusters.get(0);
      List<Point2DReadOnly> navigableExtrusionsInLocal = cluster.getNavigableExtrusionsInLocal();
      List<Point2DReadOnly> nonNavigableExtrusionsInLocal = cluster.getNonNavigableExtrusionsInLocal();
      assertEquals(12, navigableExtrusionsInLocal.size());
      assertEquals(12, nonNavigableExtrusionsInLocal.size());

      assertTrue(listContains(navigableExtrusionsInLocal, new Point2D(0.0, -0.2)));
      assertTrue(listContains(navigableExtrusionsInLocal, new Point2D(-0.2, 0.0)));
      assertTrue(listContains(navigableExtrusionsInLocal, new Point2D(0.0, 0.2)));
      assertTrue(listContains(navigableExtrusionsInLocal, new Point2D(0.1, 0.4)));
      assertTrue(listContains(navigableExtrusionsInLocal, new Point2D(0.2, 0.4)));
      assertTrue(listContains(navigableExtrusionsInLocal, new Point2D(0.6, 0.0)));
      assertTrue(listContains(navigableExtrusionsInLocal, new Point2D(0.2, -0.4)));
      assertTrue(listContains(navigableExtrusionsInLocal, new Point2D(0.1, -0.4)));
   }

   private List<Cluster> createObstacleClustersForTests(List<PlanarRegion> obstacleRegions, PlanarRegion homeRegion)
   {
      double orthogonalAngle = 0.8;

      ObstacleExtrusionDistanceCalculator extrusionDistanceCalculator = new ObstacleExtrusionDistanceCalculator()
      {
         @Override
         public double computeExtrusionDistance(Point2DReadOnly pointToExtrude, double obstacleHeight)
         {
            return 0.04;
         }
      };

      DefaultVisibilityGraphParameters parameters = new DefaultVisibilityGraphParameters();
      ObstacleRegionFilter obstacleRegionFilter = parameters.getObstacleRegionFilter();

      List<PlanarRegion> filteredObstacleRegions = obstacleRegions.stream()
                                                                  .filter(candidate -> obstacleRegionFilter.isRegionValidObstacle(candidate, homeRegion))
                                                                  .collect(Collectors.toList());

      PlanarRegionFilter planarRegionFilter = parameters.getPlanarRegionFilter();
      double DEPTH_THRESHOLD_FOR_CONVEX_DECOMPOSITION = 0.05;

      filteredObstacleRegions = PlanarRegionTools.filterRegionsByTruncatingVerticesBeneathHomeRegion(filteredObstacleRegions, homeRegion,
                                                                                                     DEPTH_THRESHOLD_FOR_CONVEX_DECOMPOSITION,
                                                                                                     planarRegionFilter);

      List<Cluster> obstacleClusters = ClusterTools.createObstacleClusters(homeRegion, filteredObstacleRegions, orthogonalAngle, extrusionDistanceCalculator);
      return obstacleClusters;
   }

   private PlanarRegion createPlanarRegion(RigidBodyTransform transform, Point2D[] points)
   {
      ConvexPolygon2D convexPolygon = createConvexPolygon(points);
      PlanarRegion planarRegion = new PlanarRegion(transform, convexPolygon);
      return planarRegion;
   }

   private PlanarRegion createPlanarRegionFromSeveralPolygons(Point2D[] concaveHullVertices, RigidBodyTransform transform, Point2D[]... listOfPoints)
   {
      ArrayList<ConvexPolygon2D> polygons = new ArrayList<>();

      for (Point2D[] points : listOfPoints)
      {
         polygons.add(createConvexPolygon(points));
      }

      PlanarRegion planarRegion = new PlanarRegion(transform, concaveHullVertices, polygons);

      return planarRegion;
   }

   private ConvexPolygon2D createConvexPolygon(Point2D[] points)
   {
      ConvexPolygon2D convexPolygon = new ConvexPolygon2D();
      for (Point2D position : points)
      {
         convexPolygon.addVertex(position);
      }
      convexPolygon.update();
      return convexPolygon;
   }

   private boolean listContainsAllXYMatch(List<Point3DReadOnly> points, Point2DReadOnly... pointsToCheck)
   {
      for (Point2DReadOnly pointToCheck : pointsToCheck)
      {
         if (!listContainsXYMatch(points, pointToCheck))
            return false;
      }

      return true;
   }

   private boolean listContainsXYMatch(List<Point3DReadOnly> points, Point2DReadOnly pointToCheck)
   {
      for (Point3DReadOnly point : points)
      {
         if (pointToCheck.epsilonEquals(new Point2D(point), EPSILON))
            return true;
      }
      return false;
   }

   private boolean listContains(List<Point2DReadOnly> points, Point2DReadOnly pointToCheck)
   {
      for (Point2DReadOnly point : points)
      {
         if (point.epsilonEquals(pointToCheck, EPSILON))
            return true;
      }
      return false;
   }

   private boolean listContains(List<? extends Point3DReadOnly> points, Point3D pointToCheck)
   {
      for (Point3DReadOnly point : points)
      {
         if (point.epsilonEquals(pointToCheck, EPSILON))
            return true;
      }
      return false;
   }

   private Point3D createAndTransformPoint(Point3D pointToTransform, RigidBodyTransform transform)
   {
      Point3D transformedPoint = new Point3D(pointToTransform);
      transform.transform(transformedPoint);
      return transformedPoint;
   }

   //TOOD: Refactor this to use Transform Tools, but need to change from FramePoint there...
   private static RigidBodyTransform createTransformFromPointAndZAxis(Point3DReadOnly point, Vector3DReadOnly zAxis)
   {
      RigidBodyTransform ret = new RigidBodyTransform();
      ret.setRotation(EuclidGeometryTools.axisAngleFromZUpToVector3D(zAxis));
      ret.setTranslation(point);
      return ret;
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(ClusterTools.class, ClusterToolsTest.class);
   }

}
