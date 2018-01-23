package us.ihmc.robotics.geometry;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;

public class ConvexPolygon2dTestHelpers
{
   public static FrameConvexPolygon2d constructPolygon(ReferenceFrame referenceFrame, double[][] points)
   {
      FrameConvexPolygon2d ret = new FrameConvexPolygon2d(referenceFrame, points);
      verifyPointsAreClockwise(ret);

      return ret;
   }

   public static void verifyPointsAreClockwise(FrameConvexPolygon2d polygon)
   {
      // Make sure points are clockwise, by taking the cross of each edge and the next clockwise edge and making sure its negative:

      int numPoints = polygon.getNumberOfVertices();

      for (int i = 0; i < numPoints; i++)
      {
         FramePoint2D point1 = polygon.getFrameVertexCopy(i);
         FramePoint2D point2 = polygon.getFrameVertexCopy((i + 1) % numPoints);
         FramePoint2D point3 = polygon.getFrameVertexCopy((i + 2) % numPoints);

         FrameVector2D vector1 = new FrameVector2D(point2);
         vector1.sub(point1);

         FrameVector2D vector2 = new FrameVector2D(point3);
         vector2.sub(point2);

         double cross = vector1.cross(vector2);

         if (cross >= 0.0)
            throw new RuntimeException("Points are not in clockwise order! FrameConvexPolygon2d:" + polygon);
      }
   }

   public static void verifyPointsAreClockwise(ArrayList<FramePoint2D> clockwisePoints)
   {
      // Make sure points are clockwise, by taking the cross of each edge and the next clockwise edge and making sure its negative:

      int numPoints = clockwisePoints.size();

      for (int i = 0; i < numPoints; i++)
      {
         FramePoint2D point1 = clockwisePoints.get(i);
         FramePoint2D point2 = clockwisePoints.get((i + 1) % numPoints);
         FramePoint2D point3 = clockwisePoints.get((i + 2) % numPoints);

         FrameVector2D vector1 = new FrameVector2D(point2);
         vector1.sub(point1);

         FrameVector2D vector2 = new FrameVector2D(point3);
         vector2.sub(point2);

         double cross = vector1.cross(vector2);

         if (cross >= 0.0)
            throw new RuntimeException("Points are not in clockwise order! Points:" + clockwisePoints);
      }
   }

   public static void verifyListContains(ArrayList<FramePoint2D> framePoints, FramePoint2D framePointToTest, double epsilon)
   {
      for (FramePoint2D framePoint : framePoints)
      {
         if (framePoint.epsilonEquals(framePointToTest, epsilon))
            return;
      }

      throw new RuntimeException("List doesn't contain " + framePointToTest);
   }

   public static void verifyPolygonContains(FrameConvexPolygon2d convexPolygon2d, FramePoint2D framePointToTest, double epsilon)
   {
      for (int i = 0; i < convexPolygon2d.getNumberOfVertices(); i++)
      {
         if (convexPolygon2d.getFrameVertexCopy(i).epsilonEquals(framePointToTest, epsilon))
            return;
      }

      throw new RuntimeException("List doesn't contain " + framePointToTest);
   }

   public static ArrayList<FrameConvexPolygon2d> generateRandomPolygons(Random random, ReferenceFrame zUpFrame, double xMin, double xMax, double yMin,
         double yMax, double widthMax, double heightMax, int numberOfPoints, int numberOfPolygons)
   {
      ArrayList<FrameConvexPolygon2d> ret = new ArrayList<FrameConvexPolygon2d>(numberOfPolygons);
      for (int i = 0; i < numberOfPolygons; i++)
      {
         FramePoint2D center = EuclidFrameRandomTools.nextFramePoint2D(random, zUpFrame, xMin, xMax, yMin, yMax);

         //       double x2A = xMin + widthMax/2.0 + random.nextDouble() * (xMax - xMin - widthMax);
         //       double x2B = xMin + widthMax/2.0 + random.nextDouble() * (xMax - xMin - widthMax);
         //
         //       double y2A = yMin + heightMax/2.0 + random.nextDouble() * (yMax - yMin - heightMax);
         //       double y2B = yMin + heightMax/2.0 + random.nextDouble() * (yMax - yMin - heightMax);
         //
         double xMin2 = center.getX() - widthMax / 2.0;
         double xMax2 = center.getX() + widthMax / 2.0;

         double yMin2 = center.getY() - heightMax / 2.0;
         double yMax2 = center.getY() + heightMax / 2.0;

         FrameConvexPolygon2d polygon = generateRandomPolygon(random, zUpFrame, xMin2, xMax2, yMin2, yMax2, numberOfPoints);

         ret.add(polygon);
      }

      return ret;
   }

   public static FrameConvexPolygon2d generateRandomPolygon(Random random, ReferenceFrame zUpFrame, double xMin, double xMax, double yMin, double yMax,
         int numberOfPoints)
   {
      FramePoint2D randomExtents1 = EuclidFrameRandomTools.nextFramePoint2D(random, zUpFrame, xMin, xMax, yMin, yMax);
      FramePoint2D randomExtents2 = EuclidFrameRandomTools.nextFramePoint2D(random, zUpFrame, xMin, xMax, yMin, yMax);

      double xMin2 = Math.min(randomExtents1.getX(), randomExtents2.getX());
      double xMax2 = Math.max(randomExtents1.getX(), randomExtents2.getX());

      double yMin2 = Math.min(randomExtents1.getY(), randomExtents2.getY());
      double yMax2 = Math.max(randomExtents1.getY(), randomExtents2.getY());

      ArrayList<FramePoint2D> points = generateRandomCircularFramePoints(random, zUpFrame, xMin2, xMax2, yMin2, yMax2, numberOfPoints);

      return new FrameConvexPolygon2d(points);
   }

   public static ArrayList<FramePoint2D> generateRandomCircularFramePoints(Random random, ReferenceFrame zUpFrame, double xMin, double xMax, double yMin,
         double yMax, int numberOfPoints)
   {
      ArrayList<FramePoint2D> points = new ArrayList<FramePoint2D>();

      FramePoint2D zeroFramePoint = new FramePoint2D(zUpFrame, (xMax + xMin) / 2.0, (yMax + yMin) / 2.0);

      for (int i = 0; i < numberOfPoints; i++)
      {
         FramePoint2D randomPoint = EuclidFrameRandomTools.nextFramePoint2D(random, zUpFrame, xMin, xMax, yMin, yMax);

         if (randomPoint.distance(zeroFramePoint) > (Math.max((xMax - xMin) / 2.0, (yMax - yMin) / 2.0)))
            continue;

         points.add(randomPoint);
      }

      return points;
   }

   public static ArrayList<FramePoint2D> generateRandomRectangularFramePoints(Random random, ReferenceFrame zUpFrame, double xMin, double xMax, double yMin,
         double yMax, int numberOfPoints)
   {
      ArrayList<FramePoint2D> points = new ArrayList<FramePoint2D>();

      for (int i = 0; i < numberOfPoints; i++)
      {
         FramePoint2D randomPoint = EuclidFrameRandomTools.nextFramePoint2D(random, zUpFrame, xMin, xMax, yMin, yMax);

         points.add(randomPoint);
      }

      return points;
   }

   public static final void verifyAroundTheCornerEdges(FrameConvexPolygon2d polygon, FramePoint2D observingPoint, FramePoint2D[] lineOfSightVertices,
         FrameLineSegment2D[] aroundTheCornerEdges)
   {
      // First make sure that the first points in the edges are the line of sight vertices:

      FramePoint2D lineOfSightPoint1 = lineOfSightVertices[0];
      FramePoint2D lineOfSightPoint2 = lineOfSightVertices[1];

      FrameLineSegment2D edge1 = aroundTheCornerEdges[0];
      FrameLineSegment2D edge2 = aroundTheCornerEdges[1];

      if (lineOfSightPoint1.distance(edge1.pointBetweenEndpointsGivenPercentage(0.0)) > 1e-7)
         throw new RuntimeException("aroundTheCornerEdge[0] does not start with the left line of sight point!");
      if (lineOfSightPoint2.distance(edge2.pointBetweenEndpointsGivenPercentage(0.0)) > 1e-7)
         throw new RuntimeException("aroundTheCornerEdge[1] does not start with the left line of sight point!");

      // Now make sure that the second point on the edge is not visible:

      FramePoint2DBasics leftPointAroundEdge = edge1.pointBetweenEndpointsGivenPercentage(1.0);
      FramePoint2DBasics rightPointAroundEdge = edge2.pointBetweenEndpointsGivenPercentage(1.0);

      double epsilon = 1.0e-5;

      FrameVector2D observingToLeftPoint = new FrameVector2D(leftPointAroundEdge);
      observingToLeftPoint.sub(observingPoint);
      observingToLeftPoint.normalize();
      observingToLeftPoint.scale(epsilon);

      FrameVector2D observingToRightPoint = new FrameVector2D(rightPointAroundEdge);
      observingToRightPoint.sub(observingPoint);
      observingToRightPoint.normalize();
      observingToRightPoint.scale(epsilon);

      FramePoint2D testPoint = new FramePoint2D(leftPointAroundEdge);
      testPoint.sub(observingToLeftPoint);

      if (!polygon.isPointInside(testPoint))
      {
         throw new RuntimeException(leftPointAroundEdge + " can be seen from the observing point! It's not around the edge!. testPoint = " + testPoint
               + " is not Inside the polygon!");
      }

      testPoint = new FramePoint2D(rightPointAroundEdge);
      testPoint.sub(observingToRightPoint);

      if (!polygon.isPointInside(testPoint))
      {
         throw new RuntimeException(rightPointAroundEdge + " can be seen from the observing point! It's not around the edge!. testPoint = " + testPoint
               + " is not Inside the polygon!");
      }
   }

   public static void verifyLineOfSightVertices(FrameConvexPolygon2d polygon, FramePoint2D observingPoint, FramePoint2D[] lineOfSightVertices)
   {
      // Point a little before and a little after the lineOfSightVertices should be outside the polygon:
      double epsilon = 1.0e-5;

      FrameVector2D observingToLineOfSight1 = new FrameVector2D(lineOfSightVertices[0]);
      observingToLineOfSight1.sub(observingPoint);
      observingToLineOfSight1.normalize();
      observingToLineOfSight1.scale(epsilon);

      FrameVector2D observingToLineOfSight2 = new FrameVector2D(lineOfSightVertices[1]);
      observingToLineOfSight2.sub(observingPoint);
      observingToLineOfSight2.normalize();
      observingToLineOfSight2.scale(epsilon);

      FramePoint2D testPoint = new FramePoint2D(lineOfSightVertices[0]);
      testPoint.add(observingToLineOfSight1);

      if (polygon.isPointInside(testPoint))
      {
         throw new RuntimeException(lineOfSightVertices[0] + " is not a line of sight vertex!");
      }

      testPoint = new FramePoint2D(lineOfSightVertices[0]);
      testPoint.sub(observingToLineOfSight1);

      if (polygon.isPointInside(testPoint))
      {
         throw new RuntimeException(lineOfSightVertices[0] + " is not a line of sight vertex!. testPoint = " + testPoint + " is Inside the polygon!");
      }

      testPoint = new FramePoint2D(lineOfSightVertices[1]);
      testPoint.add(observingToLineOfSight2);

      if (polygon.isPointInside(testPoint))
      {
         throw new RuntimeException(lineOfSightVertices[1] + " is not a line of sight vertex!");
      }

      testPoint = new FramePoint2D(lineOfSightVertices[1]);
      testPoint.sub(observingToLineOfSight2);

      if (polygon.isPointInside(testPoint))
      {
         throw new RuntimeException(lineOfSightVertices[1] + " is not a line of sight vertex!");
      }
   }

   public static void verifyLineDoesNotIntersectPolygon(FrameLine2D frameLine2d, FrameConvexPolygon2d polygon)
   {
      FramePoint2D point = new FramePoint2D(frameLine2d.getPoint());
      FramePoint2D[] lineOfSightVertices = polygon.getLineOfSightVerticesCopy(point);
      if (lineOfSightVertices == null)
      {
         throw new RuntimeException();
      }

      if (isLineStrictlyBetweenVertices(frameLine2d, lineOfSightVertices[0], lineOfSightVertices[1]))
      {
         throw new RuntimeException();
      }
   }

   public static void verifyLineIntersectsEdge(FrameLine2D frameLine2d, FrameLineSegment2D[] intersectingEdges)
   {
      FrameLineSegment2D enteringEdge, leavingEdge;

      if (intersectingEdges.length == 2)
      {
         enteringEdge = intersectingEdges[0];
         leavingEdge = intersectingEdges[1];
      }
      else
      {
         enteringEdge = null;
         leavingEdge = intersectingEdges[0];
      }

      if (enteringEdge != null)
      {
         FramePoint2DReadOnly[] enteringVertices = {enteringEdge.getFirstEndpoint(), enteringEdge.getSecondEndpoint()};

         if (!isLineBetweenOrIntersectingVertices(frameLine2d, enteringVertices[1], enteringVertices[0])
               && !isLineBetweenOrIntersectingVertices(frameLine2d, enteringVertices[0], enteringVertices[1]))
         {
            throw new RuntimeException();
         }
      }

      FramePoint2DReadOnly[] leavingVertices = {leavingEdge.getFirstEndpoint(), leavingEdge.getSecondEndpoint()};

      if (!isLineBetweenOrIntersectingVertices(frameLine2d, leavingVertices[0], leavingVertices[1])
            && !isLineBetweenOrIntersectingVertices(frameLine2d, leavingVertices[1], leavingVertices[0]))
      {
         throw new RuntimeException();
      }
   }

   public static boolean isLineBetweenOrIntersectingVertices(FrameLine2D frameLine2d, FramePoint2DReadOnly leftVertex, FramePoint2DReadOnly rightVertex)
   {
      boolean mustBeStrictlyBetween = false;

      return isLineBetweenVertices(frameLine2d, leftVertex, rightVertex, mustBeStrictlyBetween);
   }

   public static boolean isLineStrictlyBetweenVertices(FrameLine2D frameLine2d, FramePoint2D leftVertex, FramePoint2D rightVertex)
   {
      boolean mustBeStrictlyBetween = true;

      return isLineBetweenVertices(frameLine2d, leftVertex, rightVertex, mustBeStrictlyBetween);
   }

   public static boolean isLineBetweenVertices(FrameLine2D frameLine2d, FramePoint2DReadOnly leftVertex, FramePoint2DReadOnly rightVertex, boolean mustBeStrictlyBetween)
   {
      FramePoint2D lineStart = new FramePoint2D(frameLine2d.getPoint());
      FrameVector2D lineDirection = new FrameVector2D(frameLine2d.getDirection());

      double startToLeftVertexX = leftVertex.getX() - lineStart.getX();
      double startToLeftVertexY = leftVertex.getY() - lineStart.getY();

      double startToRightVertexX = rightVertex.getX() - lineStart.getX();
      double startToRightVertexY = rightVertex.getY() - lineStart.getY();

      //  double crossProduct = vectorToEdge1X * vectorToEdge2Y - vectorToEdge1Y * vectorToEdge2X;

      double leftCrossProduct = lineDirection.getX() * startToLeftVertexY - lineDirection.getY() * startToLeftVertexX;
      double rightCrossProduct = lineDirection.getX() * startToRightVertexY - lineDirection.getY() * startToRightVertexX;

      if (mustBeStrictlyBetween)
      {
         if ((leftCrossProduct > 0.0) && (rightCrossProduct < 0.0))
            return true;
      }

      else if ((leftCrossProduct >= 0.0) && (rightCrossProduct <= 0.0))
         return true;

      return false;
   }

   public static void verifyPointsAreInside(FrameConvexPolygon2d polygon, ArrayList<FramePoint2D> pointsThatShouldBeInside, double epsilon)
   {
      for (FramePoint2D point : pointsThatShouldBeInside)
      {
         if (!polygon.isPointInside(point, epsilon))
         {
            throw new RuntimeException("Point is not inside polygon. Point = " + point);
         }
      }
   }

   public static void verifyPointsAreNotInside(FrameConvexPolygon2d polygon, ArrayList<FramePoint2D> pointsThatShouldNotBeInside, double epsilon)
   {
      for (FramePoint2D point : pointsThatShouldNotBeInside)
      {
         if (polygon.isPointInside(point, epsilon))
         {
            throw new RuntimeException("Point is inside polygon. Point = " + point);
         }
      }
   }

   public static void verifyLinesIntersectPolygon(FrameConvexPolygon2d polygon, ArrayList<FrameLine2D> lines)
   {
      for (FrameLine2D line : lines)
      {
         verifyLineIntersectsPolygon(polygon, line);
      }
   }

   public static void verifyLineSegmentsIntersectPolygon(FrameConvexPolygon2d polygon, ArrayList<FrameLineSegment2D> lineSegments)
   {
      for (FrameLineSegment2D lineSegment : lineSegments)
      {
         verifyLineSegmentIntersectsPolygon(polygon, lineSegment);
      }
   }

   public static void verifyLinesDoNotIntersectPolygon(FrameConvexPolygon2d polygon, ArrayList<FrameLine2D> lines)
   {
      for (FrameLine2D line : lines)
      {
         verifyLineDoesNotIntersectsPolygon(polygon, line);
      }
   }

   public static void verifyLineSegmentsDoNotIntersectPolygon(FrameConvexPolygon2d polygon, ArrayList<FrameLineSegment2D> lineSegments)
   {
      for (FrameLineSegment2D lineSegment : lineSegments)
      {
         verifyLineSegmentDoesNotIntersectsPolygon(polygon, lineSegment);
      }
   }

   public static void verifyLineIntersectsPolygon(FrameConvexPolygon2d polygon, FrameLine2D line)
   {
      FramePoint2D[] intersectingPoints = polygon.intersectionWith(line);

      verifyPointsAreNotEmpty(intersectingPoints);
      verifyPointsAreOnLine(intersectingPoints, line);
      verifyPointsAreOnPolygon(intersectingPoints, polygon);
   }

   private static void verifyPointsAreOnPolygon(FramePoint2D[] intersectingPoints, FrameConvexPolygon2d polygon)
   {
      for (FramePoint2D point : intersectingPoints)
      {
         if (polygon.distance(point) > 1e-10)
            throw new RuntimeException("Point is not on polygon. Point = " + point);
      }

   }

   private static void verifyPointsAreNotEmpty(FramePoint2D[] points)
   {
      if (points == null || (points.length == 0))
      {
         throw new RuntimeException("Points are empty!");
      }

   }

   public static void verifyPointsAreOnLine(FramePoint2D[] intersectingPoints, FrameLine2D line)
   {
      for (FramePoint2D point : intersectingPoints)
      {
         if (line.distance(point) > 1e-5)
            throw new RuntimeException("Point is not on line. Point = " + point + ". Distance = " + line.distance(point));
      }
   }

   public static void verifyPointsAreOnLineSegment(FramePoint2D[] intersectingPoints, FrameLineSegment2D lineSegment)
   {
      for (FramePoint2D point : intersectingPoints)
      {
         if (lineSegment.distance(point) > 1e-5)
            throw new RuntimeException("Point is not on lineSegment. Point = " + point + ". Distance = " + lineSegment.distance(point));
      }
   }

   public static void verifyLineSegmentIntersectsPolygon(FrameConvexPolygon2d polygon, FrameLineSegment2D lineSegment)
   {
      FramePoint2D[] intersectingPoints = polygon.intersectionWith(lineSegment);

      verifyPointsAreNotEmpty(intersectingPoints);
      verifyPointsAreOnLineSegment(intersectingPoints, lineSegment);
      verifyPointsAreOnPolygon(intersectingPoints, polygon);
   }

   public static void verifyLineDoesNotIntersectsPolygon(FrameConvexPolygon2d polygon, FrameLine2D line)
   {
      if (doesLineIntersectPolygon(polygon, line))
      {
         throw new RuntimeException("Line intersects polygon since polygon points are on different sides of the line! line = " + line);
      }
   }

   public static boolean doesLineIntersectPolygon(FrameConvexPolygon2d polygon, FrameLine2D line)
   {
      boolean onLeft = line.isPointOnLeftSideOfLine(polygon.getFrameVertexCopy(0));

      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         if (onLeft != line.isPointOnLeftSideOfLine(polygon.getFrameVertexCopy(i)))
         {
            return true;
         }
      }

      return false;
   }

   public static void verifyLineSegmentDoesNotIntersectsPolygon(FrameConvexPolygon2d polygon, FrameLineSegment2D lineSegment)
   {
      FrameLine2D line = new FrameLine2D(lineSegment);

      if (!doesLineIntersectPolygon(polygon, line))
         return;

      FramePoint2D[] intersections = polygon.intersectionWith(line);

      for (FramePoint2D intersection : intersections)
      {
         if (lineSegment.distance(intersection) < 1e-10)
         {
            throw new RuntimeException("Line segment intersects polygon at " + intersection);
         }
      }

   }
}
