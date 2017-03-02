package us.ihmc.robotics.geometry;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

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
         FramePoint2d point1 = polygon.getFrameVertexCopy(i);
         FramePoint2d point2 = polygon.getFrameVertexCopy((i + 1) % numPoints);
         FramePoint2d point3 = polygon.getFrameVertexCopy((i + 2) % numPoints);

         FrameVector2d vector1 = new FrameVector2d(point2);
         vector1.sub(point1);

         FrameVector2d vector2 = new FrameVector2d(point3);
         vector2.sub(point2);

         double cross = vector1.cross(vector2);

         if (cross >= 0.0)
            throw new RuntimeException("Points are not in clockwise order! FrameConvexPolygon2d:" + polygon);
      }
   }

   public static void verifyPointsAreClockwise(ArrayList<FramePoint2d> clockwisePoints)
   {
      // Make sure points are clockwise, by taking the cross of each edge and the next clockwise edge and making sure its negative:

      int numPoints = clockwisePoints.size();

      for (int i = 0; i < numPoints; i++)
      {
         FramePoint2d point1 = clockwisePoints.get(i);
         FramePoint2d point2 = clockwisePoints.get((i + 1) % numPoints);
         FramePoint2d point3 = clockwisePoints.get((i + 2) % numPoints);

         FrameVector2d vector1 = new FrameVector2d(point2);
         vector1.sub(point1);

         FrameVector2d vector2 = new FrameVector2d(point3);
         vector2.sub(point2);

         double cross = vector1.cross(vector2);

         if (cross >= 0.0)
            throw new RuntimeException("Points are not in clockwise order! Points:" + clockwisePoints);
      }
   }

   public static void verifyListContains(ArrayList<FramePoint2d> framePoints, FramePoint2d framePointToTest, double epsilon)
   {
      for (FramePoint2d framePoint : framePoints)
      {
         if (framePoint.epsilonEquals(framePointToTest, epsilon))
            return;
      }

      throw new RuntimeException("List doesn't contain " + framePointToTest);
   }

   public static void verifyPolygonContains(FrameConvexPolygon2d convexPolygon2d, FramePoint2d framePointToTest, double epsilon)
   {
      for (int i = 0; i < convexPolygon2d.getNumberOfVertices(); i++)
      {
         if (convexPolygon2d.getFrameVertexCopy(i).epsilonEquals(framePointToTest, epsilon))
            return;
      }

      throw new RuntimeException("List doesn't contain " + framePointToTest);
   }

   public static void verifyOrthogonalProjection(FrameConvexPolygon2d polygon, FramePoint2d testPoint, FramePoint2d projectionPoint)
   {
      // If the test point is inside, then the projection should be itself:

      if (polygon.isPointInside(testPoint))
      {
         double distance = testPoint.distance(projectionPoint);
         if (distance > 1e-7)
            throw new RuntimeException();

         return;
      }

      // The projected point should be inside the polygon:

      if (!polygon.isPointInside(projectionPoint, 1.0E-10))
      {
         throw new RuntimeException("ProjectionPoint is not inside the polygon!");
      }

      FrameLineSegment2d[] edges = getNearestEdges(testPoint, polygon);
      int numEdges = edges.length;

      // Verify that the projection with each edge is correct:
      for (int i = 0; i < numEdges; i++)
      {
         FrameLineSegment2d edge = edges[i];

         FramePoint2d verifyProjection = edge.orthogonalProjectionCopy(testPoint);

         double distance = verifyProjection.distance(projectionPoint);

         if (distance > 1e-7)
            throw new RuntimeException("testPoint = " + testPoint + ", projectionPoint = " + projectionPoint + ", verifyProjection = " + verifyProjection);
      }

   }

   public static ArrayList<FrameConvexPolygon2d> generateRandomPolygons(Random random, ReferenceFrame zUpFrame, double xMin, double xMax, double yMin,
         double yMax, double widthMax, double heightMax, int numberOfPoints, int numberOfPolygons)
   {
      ArrayList<FrameConvexPolygon2d> ret = new ArrayList<FrameConvexPolygon2d>(numberOfPolygons);
      for (int i = 0; i < numberOfPolygons; i++)
      {
         FramePoint2d center = FramePoint2d.generateRandomFramePoint2d(random, zUpFrame, xMin, xMax, yMin, yMax);

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
      FramePoint2d randomExtents1 = FramePoint2d.generateRandomFramePoint2d(random, zUpFrame, xMin, xMax, yMin, yMax);
      FramePoint2d randomExtents2 = FramePoint2d.generateRandomFramePoint2d(random, zUpFrame, xMin, xMax, yMin, yMax);

      double xMin2 = Math.min(randomExtents1.getX(), randomExtents2.getX());
      double xMax2 = Math.max(randomExtents1.getX(), randomExtents2.getX());

      double yMin2 = Math.min(randomExtents1.getY(), randomExtents2.getY());
      double yMax2 = Math.max(randomExtents1.getY(), randomExtents2.getY());

      ArrayList<FramePoint2d> points = generateRandomCircularFramePoints(random, zUpFrame, xMin2, xMax2, yMin2, yMax2, numberOfPoints);

      return new FrameConvexPolygon2d(points);
   }

   public static ArrayList<FramePoint2d> generateRandomCircularFramePoints(Random random, ReferenceFrame zUpFrame, double xMin, double xMax, double yMin,
         double yMax, int numberOfPoints)
   {
      ArrayList<FramePoint2d> points = new ArrayList<FramePoint2d>();

      FramePoint2d zeroFramePoint = new FramePoint2d(zUpFrame, (xMax + xMin) / 2.0, (yMax + yMin) / 2.0);

      for (int i = 0; i < numberOfPoints; i++)
      {
         FramePoint2d randomPoint = FramePoint2d.generateRandomFramePoint2d(random, zUpFrame, xMin, xMax, yMin, yMax);

         if (randomPoint.distance(zeroFramePoint) > (Math.max((xMax - xMin) / 2.0, (yMax - yMin) / 2.0)))
            continue;

         points.add(randomPoint);
      }

      return points;
   }

   public static ArrayList<FramePoint2d> generateRandomRectangularFramePoints(Random random, ReferenceFrame zUpFrame, double xMin, double xMax, double yMin,
         double yMax, int numberOfPoints)
   {
      ArrayList<FramePoint2d> points = new ArrayList<FramePoint2d>();

      for (int i = 0; i < numberOfPoints; i++)
      {
         FramePoint2d randomPoint = FramePoint2d.generateRandomFramePoint2d(random, zUpFrame, xMin, xMax, yMin, yMax);

         points.add(randomPoint);
      }

      return points;
   }

   public static final void verifyAroundTheCornerEdges(FrameConvexPolygon2d polygon, FramePoint2d observingPoint, FramePoint2d[] lineOfSightVertices,
         FrameLineSegment2d[] aroundTheCornerEdges)
   {
      // First make sure that the first points in the edges are the line of sight vertices:

      FramePoint2d lineOfSightPoint1 = lineOfSightVertices[0];
      FramePoint2d lineOfSightPoint2 = lineOfSightVertices[1];

      FrameLineSegment2d edge1 = aroundTheCornerEdges[0];
      FrameLineSegment2d edge2 = aroundTheCornerEdges[1];

      if (lineOfSightPoint1.distance(edge1.pointBetweenEndPointsGivenParameter(0.0)) > 1e-7)
         throw new RuntimeException("aroundTheCornerEdge[0] does not start with the left line of sight point!");
      if (lineOfSightPoint2.distance(edge2.pointBetweenEndPointsGivenParameter(0.0)) > 1e-7)
         throw new RuntimeException("aroundTheCornerEdge[1] does not start with the left line of sight point!");

      // Now make sure that the second point on the edge is not visible:

      FramePoint2d leftPointAroundEdge = edge1.pointBetweenEndPointsGivenParameter(1.0);
      FramePoint2d rightPointAroundEdge = edge2.pointBetweenEndPointsGivenParameter(1.0);

      double epsilon = 1.0e-5;

      FrameVector2d observingToLeftPoint = new FrameVector2d(leftPointAroundEdge);
      observingToLeftPoint.sub(observingPoint);
      observingToLeftPoint.normalize();
      observingToLeftPoint.scale(epsilon);

      FrameVector2d observingToRightPoint = new FrameVector2d(rightPointAroundEdge);
      observingToRightPoint.sub(observingPoint);
      observingToRightPoint.normalize();
      observingToRightPoint.scale(epsilon);

      FramePoint2d testPoint = new FramePoint2d(leftPointAroundEdge);
      testPoint.sub(observingToLeftPoint);

      if (!polygon.isPointInside(testPoint))
      {
         throw new RuntimeException(leftPointAroundEdge + " can be seen from the observing point! It's not around the edge!. testPoint = " + testPoint
               + " is not Inside the polygon!");
      }

      testPoint = new FramePoint2d(rightPointAroundEdge);
      testPoint.sub(observingToRightPoint);

      if (!polygon.isPointInside(testPoint))
      {
         throw new RuntimeException(rightPointAroundEdge + " can be seen from the observing point! It's not around the edge!. testPoint = " + testPoint
               + " is not Inside the polygon!");
      }
   }

   public static void verifyLineOfSightVertices(FrameConvexPolygon2d polygon, FramePoint2d observingPoint, FramePoint2d[] lineOfSightVertices)
   {
      // Point a little before and a little after the lineOfSightVertices should be outside the polygon:
      double epsilon = 1.0e-5;

      FrameVector2d observingToLineOfSight1 = new FrameVector2d(lineOfSightVertices[0]);
      observingToLineOfSight1.sub(observingPoint);
      observingToLineOfSight1.normalize();
      observingToLineOfSight1.scale(epsilon);

      FrameVector2d observingToLineOfSight2 = new FrameVector2d(lineOfSightVertices[1]);
      observingToLineOfSight2.sub(observingPoint);
      observingToLineOfSight2.normalize();
      observingToLineOfSight2.scale(epsilon);

      FramePoint2d testPoint = new FramePoint2d(lineOfSightVertices[0]);
      testPoint.add(observingToLineOfSight1);

      if (polygon.isPointInside(testPoint))
      {
         throw new RuntimeException(lineOfSightVertices[0] + " is not a line of sight vertex!");
      }

      testPoint = new FramePoint2d(lineOfSightVertices[0]);
      testPoint.sub(observingToLineOfSight1);

      if (polygon.isPointInside(testPoint))
      {
         throw new RuntimeException(lineOfSightVertices[0] + " is not a line of sight vertex!. testPoint = " + testPoint + " is Inside the polygon!");
      }

      testPoint = new FramePoint2d(lineOfSightVertices[1]);
      testPoint.add(observingToLineOfSight2);

      if (polygon.isPointInside(testPoint))
      {
         throw new RuntimeException(lineOfSightVertices[1] + " is not a line of sight vertex!");
      }

      testPoint = new FramePoint2d(lineOfSightVertices[1]);
      testPoint.sub(observingToLineOfSight2);

      if (polygon.isPointInside(testPoint))
      {
         throw new RuntimeException(lineOfSightVertices[1] + " is not a line of sight vertex!");
      }
   }

   public static void verifyLineDoesNotIntersectPolygon(FrameLine2d frameLine2d, FrameConvexPolygon2d polygon)
   {
      FramePoint2d point = new FramePoint2d();
      frameLine2d.getFramePoint2d(point);
      FramePoint2d[] lineOfSightVertices = polygon.getLineOfSightVerticesCopy(point);
      if (lineOfSightVertices == null)
      {
         throw new RuntimeException();
      }

      if (isLineStrictlyBetweenVertices(frameLine2d, lineOfSightVertices[0], lineOfSightVertices[1]))
      {
         throw new RuntimeException();
      }
   }

   public static void verifyLineIntersectsEdge(FrameLine2d frameLine2d, FrameLineSegment2d[] intersectingEdges)
   {
      FrameLineSegment2d enteringEdge, leavingEdge;

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
         FramePoint2d[] enteringVertices = enteringEdge.getEndFramePointsCopy();

         if (!isLineBetweenOrIntersectingVertices(frameLine2d, enteringVertices[1], enteringVertices[0])
               && !isLineBetweenOrIntersectingVertices(frameLine2d, enteringVertices[0], enteringVertices[1]))
         {
            throw new RuntimeException();
         }
      }

      FramePoint2d[] leavingVertices = leavingEdge.getEndFramePointsCopy();

      if (!isLineBetweenOrIntersectingVertices(frameLine2d, leavingVertices[0], leavingVertices[1])
            && !isLineBetweenOrIntersectingVertices(frameLine2d, leavingVertices[1], leavingVertices[0]))
      {
         throw new RuntimeException();
      }
   }

   public static boolean isLineBetweenOrIntersectingVertices(FrameLine2d frameLine2d, FramePoint2d leftVertex, FramePoint2d rightVertex)
   {
      boolean mustBeStrictlyBetween = false;

      return isLineBetweenVertices(frameLine2d, leftVertex, rightVertex, mustBeStrictlyBetween);
   }

   public static boolean isLineStrictlyBetweenVertices(FrameLine2d frameLine2d, FramePoint2d leftVertex, FramePoint2d rightVertex)
   {
      boolean mustBeStrictlyBetween = true;

      return isLineBetweenVertices(frameLine2d, leftVertex, rightVertex, mustBeStrictlyBetween);
   }

   public static boolean isLineBetweenVertices(FrameLine2d frameLine2d, FramePoint2d leftVertex, FramePoint2d rightVertex, boolean mustBeStrictlyBetween)
   {
      FramePoint2d lineStart = new FramePoint2d();
      frameLine2d.getFramePoint2d(lineStart);
      FrameVector2d lineDirection = new FrameVector2d();
      frameLine2d.getNormalizedFrameVector(lineDirection);

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

   public static void verifyPointsAreInside(FrameConvexPolygon2d polygon, ArrayList<FramePoint2d> pointsThatShouldBeInside, double epsilon)
   {
      for (FramePoint2d point : pointsThatShouldBeInside)
      {
         if (!polygon.isPointInside(point, epsilon))
         {
            throw new RuntimeException("Point is not inside polygon. Point = " + point);
         }
      }
   }

   public static void verifyPointsAreNotInside(FrameConvexPolygon2d polygon, ArrayList<FramePoint2d> pointsThatShouldNotBeInside, double epsilon)
   {
      for (FramePoint2d point : pointsThatShouldNotBeInside)
      {
         if (polygon.isPointInside(point, epsilon))
         {
            throw new RuntimeException("Point is inside polygon. Point = " + point);
         }
      }
   }

   public static void verifyLinesIntersectPolygon(FrameConvexPolygon2d polygon, ArrayList<FrameLine2d> lines)
   {
      for (FrameLine2d line : lines)
      {
         verifyLineIntersectsPolygon(polygon, line);
      }
   }

   public static void verifyLineSegmentsIntersectPolygon(FrameConvexPolygon2d polygon, ArrayList<FrameLineSegment2d> lineSegments)
   {
      for (FrameLineSegment2d lineSegment : lineSegments)
      {
         verifyLineSegmentIntersectsPolygon(polygon, lineSegment);
      }
   }

   public static void verifyLinesDoNotIntersectPolygon(FrameConvexPolygon2d polygon, ArrayList<FrameLine2d> lines)
   {
      for (FrameLine2d line : lines)
      {
         verifyLineDoesNotIntersectsPolygon(polygon, line);
      }
   }

   public static void verifyLineSegmentsDoNotIntersectPolygon(FrameConvexPolygon2d polygon, ArrayList<FrameLineSegment2d> lineSegments)
   {
      for (FrameLineSegment2d lineSegment : lineSegments)
      {
         verifyLineSegmentDoesNotIntersectsPolygon(polygon, lineSegment);
      }
   }

   public static void verifyLineIntersectsPolygon(FrameConvexPolygon2d polygon, FrameLine2d line)
   {
      FramePoint2d[] intersectingPoints = polygon.intersectionWith(line);

      verifyPointsAreNotEmpty(intersectingPoints);
      verifyPointsAreOnLine(intersectingPoints, line);
      verifyPointsAreOnPolygon(intersectingPoints, polygon);
   }

   private static void verifyPointsAreOnPolygon(FramePoint2d[] intersectingPoints, FrameConvexPolygon2d polygon)
   {
      for (FramePoint2d point : intersectingPoints)
      {
         if (polygon.distance(point) > 1e-10)
            throw new RuntimeException("Point is not on polygon. Point = " + point);
      }

   }

   private static void verifyPointsAreNotEmpty(FramePoint2d[] points)
   {
      if (points == null || (points.length == 0))
      {
         throw new RuntimeException("Points are empty!");
      }

   }

   public static void verifyPointsAreOnLine(FramePoint2d[] intersectingPoints, FrameLine2d line)
   {
      for (FramePoint2d point : intersectingPoints)
      {
         if (line.distance(point) > 1e-5)
            throw new RuntimeException("Point is not on line. Point = " + point + ". Distance = " + line.distance(point));
      }
   }

   public static void verifyPointsAreOnLineSegment(FramePoint2d[] intersectingPoints, FrameLineSegment2d lineSegment)
   {
      for (FramePoint2d point : intersectingPoints)
      {
         if (lineSegment.distance(point) > 1e-5)
            throw new RuntimeException("Point is not on lineSegment. Point = " + point + ". Distance = " + lineSegment.distance(point));
      }
   }

   public static void verifyLineSegmentIntersectsPolygon(FrameConvexPolygon2d polygon, FrameLineSegment2d lineSegment)
   {
      FramePoint2d[] intersectingPoints = polygon.intersectionWith(lineSegment);

      verifyPointsAreNotEmpty(intersectingPoints);
      verifyPointsAreOnLineSegment(intersectingPoints, lineSegment);
      verifyPointsAreOnPolygon(intersectingPoints, polygon);
   }

   public static void verifyLineDoesNotIntersectsPolygon(FrameConvexPolygon2d polygon, FrameLine2d line)
   {
      if (doesLineIntersectPolygon(polygon, line))
      {
         throw new RuntimeException("Line intersects polygon since polygon points are on different sides of the line! line = " + line);
      }
   }

   public static boolean doesLineIntersectPolygon(FrameConvexPolygon2d polygon, FrameLine2d line)
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

   public static void verifyLineSegmentDoesNotIntersectsPolygon(FrameConvexPolygon2d polygon, FrameLineSegment2d lineSegment)
   {
      FrameLine2d line = new FrameLine2d(lineSegment);

      if (!doesLineIntersectPolygon(polygon, line))
         return;

      FramePoint2d[] intersections = polygon.intersectionWith(line);

      for (FramePoint2d intersection : intersections)
      {
         if (lineSegment.distance(intersection) < 1e-10)
         {
            throw new RuntimeException("Line segment intersects polygon at " + intersection);
         }
      }

   }

   public static FrameLineSegment2d[] getNearestEdges(FramePoint2d testPoint, FrameConvexPolygon2d polygon)
   {
      polygon.checkReferenceFrameMatch(testPoint);

      LineSegment2d[] edges = getNearestEdges(testPoint.getPoint(), polygon.getConvexPolygon2d());

      FrameLineSegment2d[] ret = new FrameLineSegment2d[edges.length];

      for (int i = 0; i < edges.length; i++)
      {
         ret[i] = new FrameLineSegment2d(polygon.getReferenceFrame(), edges[i]);
      }

      return ret;
   }

   public static LineSegment2d[] getNearestEdges(Point2D testPoint, ConvexPolygon2d polygon)
   {
      int[] tempTwoIndices = new int[2];

      polygon.checkIfUpToDate();
      if (!polygon.hasAtLeastTwoVertices())
      {
         return null;
      }

      int numberOfEdges = getNearestEdgeIndices(testPoint, tempTwoIndices, polygon);
      if (numberOfEdges == 0)
         return null;

      LineSegment2d[] ret = new LineSegment2d[numberOfEdges];
      for (int i = 0; i < numberOfEdges; i++)
      {
         int edgeIndex = tempTwoIndices[i];
         LineSegment2d edge = new LineSegment2d(polygon.getVertex(edgeIndex), polygon.getNextVertex(edgeIndex));

         ret[i] = edge;
      }

      return ret;
   }

   private static int getNearestEdgeIndices(Point2D pointToProject, int[] indicesToPack, ConvexPolygon2d polygon)
   {
      int[] tempTwoIndices = new int[2];

      if (indicesToPack.length != 2)
         throw new RuntimeException("Expected array of length two");

      // First find the line of sight vertices. If inside the Polygon, then return null.
      if (!ConvexPolygon2dCalculator.getLineOfSightVertexIndices(pointToProject, tempTwoIndices, polygon))
      {
         indicesToPack[0] = -1;
         indicesToPack[1] = -1;
         return 0;
      }

      int numberOfVertices = polygon.getNumberOfVertices();
      int leftEdge = (tempTwoIndices[0] - 1 + numberOfVertices) % numberOfVertices;
      int rightEdge = tempTwoIndices[1];

      // Binary search maintaining nearest left and nearest right vertices until they are adjacent: //TODO remove the q from maintaining
      while ((rightEdge != leftEdge) && (polygon.getNextVertexIndex(rightEdge) != leftEdge))
      {
         int testEdge = ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(leftEdge, rightEdge, polygon);

         if (isEdgeFullyToTheLeftOfObserver(testEdge, pointToProject, polygon))
         {
            leftEdge = testEdge;
         }
         else if (isEdgeFullyToTheRight(testEdge, pointToProject, polygon))
         {
            rightEdge = testEdge;
         }
         else
         {
            indicesToPack[0] = testEdge;
            indicesToPack[1] = -1;
            return 1;
         }
      }

      // If edges are the same, then just return one,
      if (leftEdge == rightEdge)
      {
         indicesToPack[0] = leftEdge;
         indicesToPack[1] = -1;
         return 1;
      }

      // Otherwise check if left is fully to the left or right is fully to the right:
      boolean leftEdgeIsFullyLeft = isEdgeFullyToTheLeftOfObserver(leftEdge, pointToProject, polygon);
      boolean rightEdgeIsFullyRight = isEdgeFullyToTheRight(rightEdge, pointToProject, polygon);

      // They should never both be not fully left or right...
      if (!leftEdgeIsFullyLeft && !rightEdgeIsFullyRight)
      {
         throw new RuntimeException("Should never get here!");
      }

      if (!leftEdgeIsFullyLeft)
      {
         indicesToPack[0] = leftEdge;
         indicesToPack[1] = -1;
         return 1;
      }
      if (!rightEdgeIsFullyRight)
      {
         indicesToPack[0] = rightEdge;
         indicesToPack[1] = -1;
         return 1;
      }

      indicesToPack[0] = leftEdge;
      indicesToPack[1] = rightEdge;
      return 2;
   }

   /**
    * Check if when projected on the edge line, the observer point is located at the left (when looking from inside the polygon) outside of the edge.
    * @param edgeIndex refers to the index of the first vertex of the edge.
    * @param observerPoint2d point that is compared with the edge.
    * @return true if the observer point is located at the left outside of the edge.
    */
   private static boolean isEdgeFullyToTheLeftOfObserver(int edgeIndex, Point2D observerPoint2d, ConvexPolygon2d polygon)
   {
      Point2DReadOnly vertex = polygon.getVertex(edgeIndex);
      Point2DReadOnly nextVertex = polygon.getNextVertex(edgeIndex);

      // Vector perpendicular to the edge and pointing outside the polygon
      double edgeNormalX = -(nextVertex.getY() - vertex.getY());
      double edgeNormalY = (nextVertex.getX() - vertex.getX());

      double vertexToObserverX = observerPoint2d.getX() - vertex.getX();
      double vertexToObserverY = observerPoint2d.getY() - vertex.getY();

      // Equivalent to looking at the sign of the angle formed by (egdeNormal -> vertexToPoint)
      double crossProduct = edgeNormalX * vertexToObserverY - edgeNormalY * vertexToObserverX;

      // If positive, that means vertexToPoint is pointing outside the edge
      // Thus, the observer point is totally on the left of the edge
      return crossProduct > 0.0;
   }

   /**
    * Check if when projected on the edge line, the observer point is located at the right (when looking from inside the polygon) outside of the edge.
    * @param edgeIndex refers to the index of the first vertex of the edge.
    * @param observerPoint2d point that is compared with the edge.
    * @return true if the observer point is located at the right outside of the edge.
    */
   private static boolean isEdgeFullyToTheRight(int edgeIndex, Point2D observerPoint2d, ConvexPolygon2d polygon)
   {
      Point2DReadOnly vertex = polygon.getVertex(edgeIndex);
      Point2DReadOnly nextVertex = polygon.getNextVertex(edgeIndex);

      // Vector perpendicular to the edge and pointing outside the polygon
      double edgeNormalX = -(nextVertex.getY() - vertex.getY());
      double edgeNormalY = (nextVertex.getX() - vertex.getX());

      double nextVertexToObserverX = observerPoint2d.getX() - nextVertex.getX();
      double nextVertexToObserverY = observerPoint2d.getY() - nextVertex.getY();

      // Equivalent to looking at the sign of the angle formed by (egdeNormal -> vertexToPoint)
      double crossProduct = edgeNormalX * nextVertexToObserverY - edgeNormalY * nextVertexToObserverX;

      // If positive, that means vertexToPoint is pointing outside the edge
      // Thus, the observer point is totally on the left of the edge
      return crossProduct < 0.0;
   }

}
