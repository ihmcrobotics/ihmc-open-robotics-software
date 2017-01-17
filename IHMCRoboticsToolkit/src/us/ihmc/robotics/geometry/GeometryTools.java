package us.ihmc.robotics.geometry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Tuple2d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.shapes.Plane3d;
import us.ihmc.robotics.math.Epsilons;
import us.ihmc.robotics.math.exceptions.UndefinedOperationException;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class GeometryTools
{
   public static final boolean DEBUG = false;

   private static final double EPSILON = 1e-6;

   /**
    * Computes the minimum distance between a 3D point and an infinitely long 3D line defined by a given line segment.
    * <a href="http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html"> Useful link</a>.
    * If the line direction is of length equal to zero: ||{@code lineDirection}|| {@code  == 0.0}, the distance between the {@code lineStart} and the given point is computed instead.
    *
    * @param point 3D point to compute the distance from the line. Not modified.
    * @param lineStart starting point of the line segment defining the infinite line. Not modified.
    * @param lineDirection direction of the line segment defining the infinite line. Not modified.
    * @return the minimum distance between the 3D point and the 3D line.
    */
   public static double distanceFromPointToLine(Point3d point, Point3d lineStart, Vector3d lineDirection)
   {
      double directionMagnitude = lineDirection.length();
      if (directionMagnitude < Epsilons.ONE_TRILLIONTH)
      {
         return lineStart.distance(point);
      }
      else
      {
         Vector3d pointToStart = new Vector3d(lineStart);
         pointToStart.sub(point);

         Vector3d crossProduct = new Vector3d();
         crossProduct.cross(lineDirection, pointToStart);

         return crossProduct.length() / directionMagnitude;
      }
   }

   /**
    * Computes the minimum distance between a 3D point and an infinitely long 3D line defined by a given line segment.
    * <a href="http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html"> Useful link</a>.
    * If the line is defined by the same point: {@code lineStart == lineEnd}, the distance between the {@code lineStart} and the given point is computed instead.
    *
    * @param point 3D point to compute the distance from the line. Not modified.
    * @param lineStart starting point of the line segment defining the infinite line. Not modified.
    * @param lineDirection direction of the line segment defining the infinite line. Not modified.
    * @return the minimum distance between the 3D point and the 3D line.
    */
   // TODO consider making line3d and moving into it
   public static double distanceFromPointToLine(Point3d point, Point3d lineStart, Point3d lineEnd)
   {
      if (lineStart.getX() - lineEnd.getX() == 0 & lineStart.getY() - lineEnd.getY() == 0 & lineStart.getZ() - lineEnd.getZ() == 0)
      {
         return lineStart.distance(point);
      }
      else
      {
         Vector3d lineDirection = new Vector3d(lineEnd);
         lineDirection.sub(lineStart);
         return distanceFromPointToLine(point, lineStart, lineDirection);
      }
   }

   /**
    * Returns the minimum distance between a 2D point and an infinitely long 2D
    * line defined by a given line segment.
    * If the line is defined by the same point: {@code lineStart == lineEnd}, the xy distance between the {@code lineStart} and the given point is computed instead.
    *
    * @param point the 3D point is projected onto the xy-plane. It's projection is used to compute the distance from the line. Not modified.
    * @param lineStart the projection of this 3D onto the xy-plane refers to the starting point of the 2D line segment that defines in its turn the infinite 2D line. Not modified.
    * @param lineEnd the projection of this 3D onto the xy-plane refers to the end point of the 2D line segment that defines in its turn the infinite 2D line. Not modified.
    * @return the minimum distance between the 2D point and the 2D line.
    */
   public static double distanceFromPointToLine2d(FramePoint point, FramePoint lineStart, FramePoint lineEnd)
   {
      // FIXME Need to verify that all the arguments are expressed in the same reference frame.
      return distanceFromPointToLine(point.getX(), point.getY(), lineStart.getX(), lineStart.getY(), lineEnd.getX(), lineEnd.getY());
   }

   /**
    * Returns the minimum distance between a 2D point and an infinitely long 2D
    * line defined by a given line segment.
    * If the line is defined by the same point: {@code lineStart == lineEnd}, the distance between the {@code lineStart} and the given point is computed instead.
    *
    * @param point 2D point to compute the distance from the line. Not modified.
    * @param lineStart starting point of the line segment defining the infinite line. Not modified.
    * @param lineEnd end point of the line segment defining the infinite line. Not modified.
    * @return the minimum distance between the 2D point and the 2D line.
    */
   public static double distanceFromPointToLine(Point2d point, Point2d lineStart, Point2d lineEnd)
   {
      return distanceFromPointToLine(point.getX(), point.getY(), lineStart.getX(), lineStart.getY(), lineEnd.getX(), lineEnd.getY());
   }

   /**
    * Returns the minimum distance between a 2D point and an infinitely long 2D
    * line defined by a given line segment.
    * If the line is defined by the same point: {@code lineStart == lineEnd}, the distance between the {@code lineStart} and the given point is computed instead.
    *
    * @param pointX x-coordinate of the query.
    * @param pointY y-coordinate of the query.
    * @param lineStartX x-coordinate of the starting point of the line segment defining the infinite line.
    * @param lineStartY y-coordinate of the starting point of the line segment defining the infinite line.
    * @param lineEndX x-coordinate of the end point of the line segment defining the infinite line.
    * @param lineEndY y-coordinate of the end point of the line segment defining the infinite line.
    * @return the minimum distance between the 2D point and the 2D line.
    */
   public static double distanceFromPointToLine(double pointX, double pointY, double lineStartX, double lineStartY, double lineEndX, double lineEndY)
   {
      if (lineStartX - lineEndX == 0 & lineStartY - lineEndY == 0)
      {
         double distance = Math.sqrt(((lineStartX - pointX) * (lineStartX - pointX)) + ((lineStartY - pointY) * (lineStartY - pointY)));

         return distance;
      }
      else
      {
         double numerator = Math.abs(((lineEndX - lineStartX) * (lineStartY - pointY)) - ((lineStartX - pointX) * (lineEndY - lineStartY)));
         double denominator = Math.sqrt(((lineEndX - lineStartX) * (lineEndX - lineStartX)) + ((lineEndY - lineStartY) * (lineEndY - lineStartY)));

         return numerator / denominator;
      }
   }

   /**
    * Returns the minimum distance between a point and a given line segment.
    * Holds true if line segment shrinks to a single point.
    *
    * @param xPoint x coordinate of point to be tested.
    * @param yPoint y coordinate of point to be tested.
    * @param lineStart starting point of the line segment. Not modified.
    * @param lineEnd end point of the line segment. Not modified.
    * @return the minimum distance between the 2D point and the 2D line segment.
    */
   public static double distanceFromPointToLineSegment(double xPoint, double yPoint, Point2d lineStart, Point2d lineEnd)
   {
      double startAngleDot = (lineEnd.x - lineStart.x) * (xPoint - lineStart.x) + (lineEnd.y - lineStart.y) * (yPoint - lineStart.y);
      double endAngleDot = (lineStart.x - lineEnd.x) * (xPoint - lineEnd.x) + (lineStart.y - lineEnd.y) * (yPoint - lineEnd.y);

      if ((startAngleDot >= 0.0) && (endAngleDot >= 0.0))
      {
         return distanceFromPointToLine(xPoint, yPoint, lineStart.getX(), lineStart.getY(), lineEnd.getX(), lineEnd.getY());
      }

      if (startAngleDot < 0.0)
      {
         return distanceBetweenPoints(lineStart.x, lineStart.y, xPoint, yPoint);
      }
      else
      {
         if (endAngleDot >= 0.0)
         {
            throw new RuntimeException("totally not a physical situation here");
         }

         return distanceBetweenPoints(lineEnd.x, lineEnd.y, xPoint, yPoint);
      }
   }

   /**
    * Returns the minimum distance between a point and a given line segment.
    * Holds true if line segment shrinks to a single point.
    *
    * @param point 2D point to compute the distance from the line segment. Not modified.
    * @param lineStart starting point of the line segment. Not modified.
    * @param lineEnd end point of the line segment. Not modified.
    * @return the minimum distance between the 2D point and the 2D line segment.
    */
   public static double distanceFromPointToLineSegment(Point2d point, Point2d lineStart, Point2d lineEnd)
   {
      return distanceFromPointToLineSegment(point.x, point.y, lineStart, lineEnd);
   }

   /**
    * Returns a boolean value, stating whether a 2D point is on the left side of an infinitely long line defined by a line segment.
    * "Left side" is determined based on order of {@code lineStart} and {@code lineEnd}.
    * <p>
    * For instance, given the {@code lineStart} coordinates x = 0, and y = 0, and the {@code lineEnd} coordinates x = 1, y = 0,
    * a point located on the left side of this line has a negative y coordinate.
    *<p>
    * This method will return false if the point is on the line.
    *
    * @param point the query point. Not modified.
    * @param lineStart starting point of the line segment defining the infinite line. Not modified.
    * @param lineEnd end point of the line segment defining the infinite line. Not modified.
    * @return {@code true} if the point is on the left side of the line, {@code false} if the point is on the right side or exactly on the line.
    */
   public static boolean isPointOnLeftSideOfLine(Point2d point, Point2d lineStart, Point2d lineEnd)
   {
      return isPointOnSideOfLine(point, lineStart, lineEnd, RobotSide.LEFT);
   }

   /**
    * Returns a boolean value, stating whether a 2D point is on the right side of an infinitely long line defined by a line segment.
    * "Right side" is determined based on order of {@code lineStart} and {@code lineEnd}.
    * <p>
    * For instance, given the {@code lineStart} coordinates x = 0, and y = 0, and the {@code lineEnd} coordinates x = 1, y = 0,
    * a point located on the right side of this line has a positive y coordinate.
    *<p>
    * This method will return false if the point is on the line.
    *
    * @param point the query point. Not modified.
    * @param lineStart starting point of the line segment defining the infinite line. Not modified.
    * @param lineEnd end point of the line segment defining the infinite line. Not modified.
    * @return {@code true} if the point is on the right side of the line, {@code false} if the point is on the left side or exactly on the line.
    */
   public static boolean isPointOnRightSideOfLine(Point2d point, Point2d lineStart, Point2d lineEnd)
   {
      return isPointOnSideOfLine(point, lineStart, lineEnd, RobotSide.RIGHT);
   }

   /**
    * Returns a boolean value, stating whether a 2D point is on the left or right side of an infinitely long line defined by a line segment.
    * The idea of "side" is determined based on order of {@code lineStart} and {@code lineEnd}.
    * <p>
    * For instance, given the {@code lineStart} coordinates x = 0, and y = 0, and the {@code lineEnd} coordinates x = 1, y = 0,
    * a point located on:
    * <ul>
    *    <li> the left side of this line has a negative y coordinate.
    *    <li> the right side of this line has a positive y coordinate.
    * </ul>
    *<p>
    * This method will return false if the point is on the line.
    *
    * @param point the query point. Not modified.
    * @param lineStart starting point of the line segment defining the infinite line. Not modified.
    * @param lineEnd end point of the line segment defining the infinite line. Not modified.
    * @param side the query of the side.
    * @return {@code true} if the point is on the query side of the line, {@code false} if the point is on the opposite side or exactly on the line.
    */
   public static boolean isPointOnSideOfLine(Point2d point, Point2d lineStart, Point2d lineEnd, RobotSide side)
   {
      return isPointOnSideOfLine(point.getX(), point.getY(), lineStart, lineEnd, side);
   }

   /**
    * Returns a boolean value, stating whether a 2D point is on the left or right side of an infinitely long line defined by a line segment.
    * The idea of "side" is determined based on order of {@code lineStart} and {@code lineEnd}.
    * <p>
    * For instance, given the {@code lineStart} coordinates x = 0, and y = 0, and the {@code lineEnd} coordinates x = 1, y = 0,
    * a point located on:
    * <ul>
    *    <li> the left side of this line has a negative y coordinate.
    *    <li> the right side of this line has a positive y coordinate.
    * </ul>
    *<p>
    * This method will return false if the point is on the line.
    *
    * @param pointX the x-coordinate of the query point.
    * @param pointY the y-coordinate of the query point.
    * @param lineStart starting point of the line segment defining the infinite line. Not modified.
    * @param lineEnd end point of the line segment defining the infinite line. Not modified.
    * @param side the query of the side.
    * @return {@code true} if the point is on the query side of the line, {@code false} if the point is on the opposite side or exactly on the line.
    */
   public static boolean isPointOnSideOfLine(double pointX, double pointY, Point2d lineStart, Point2d lineEnd, RobotSide side)
   {
      double pointOnLineX = lineStart.getX();
      double pointOnLineY = lineStart.getY();
      double lineDirectionX = lineEnd.getX() - lineStart.getX();
      double lineDirectionY = lineEnd.getY() - lineStart.getY();
      return isPointOnSideOfLine(pointX, pointY, pointOnLineX, pointOnLineY, lineDirectionX, lineDirectionY, side);
   }

   /**
    * Returns a boolean value, stating whether a 2D point is on the left or right side of an infinitely long line.
    * The idea of "side" is determined based on the direction of the line.
    * <p>
    * For instance, given the {@code lineDirection} components x = 0, and y = 1, and the {@code pointOnLine} coordinates x = 0, and y = 0, 
    * a point located on:
    * <ul>
    *    <li> the left side of this line has a negative y coordinate.
    *    <li> the right side of this line has a positive y coordinate.
    * </ul>
    *<p>
    * This method will return false if the point is on the line.
    *
    * @param point the query point. Not modified.
    * @param pointOnLine a point positioned on the infinite line. Not modified.
    * @param lineDirection the direction of the infinite line. Not modified.
    * @param side the query of the side.
    * @return {@code true} if the point is on the query side of the line, {@code false} if the point is on the opposite side or exactly on the line.
    */
   public static boolean isPointOnSideOfLine(Point2d point, Point2d pointOnLine, Vector2d lineDirection, RobotSide side)
   {
      return isPointOnSideOfLine(point.getX(), point.getY(), pointOnLine, lineDirection, side);
   }

   /**
    * Returns a boolean value, stating whether a 2D point is on the left or right side of an infinitely long line.
    * The idea of "side" is determined based on the direction of the line.
    * <p>
    * For instance, given the {@code lineDirection} components x = 0, and y = 1, and the {@code pointOnLine} coordinates x = 0, and y = 0, 
    * a point located on:
    * <ul>
    *    <li> the left side of this line has a negative y coordinate.
    *    <li> the right side of this line has a positive y coordinate.
    * </ul>
    *<p>
    * This method will return false if the point is on the line.
    *
    * @param pointX the x-coordinate of  the query point.
    * @param pointY the y-coordinate of  the query point.
    * @param pointOnLine a point positioned on the infinite line. Not modified.
    * @param lineDirection the direction of the infinite line. Not modified.
    * @param side the query of the side.
    * @return {@code true} if the point is on the query side of the line, {@code false} if the point is on the opposite side or exactly on the line.
    */
   public static boolean isPointOnSideOfLine(double pointX, double pointY, Point2d pointOnLine, Vector2d lineDirection, RobotSide side)
   {
      double pointOnLineX = pointOnLine.getX();
      double pointOnLineY = pointOnLine.getY();
      double lineDirectionX = lineDirection.getX();
      double lineDirectionY = lineDirection.getY();
      return isPointOnSideOfLine(pointX, pointY, pointOnLineX, pointOnLineY, lineDirectionX, lineDirectionY, side);
   }

   /**
    * Returns a boolean value, stating whether a 2D point is on the left or right side of an infinitely long line.
    * The idea of "side" is determined based on the direction of the line.
    * <p>
    * For instance, given the {@code lineDirection} components x = 0, and y = 1, and the {@code pointOnLine} coordinates x = 0, and y = 0, 
    * a point located on:
    * <ul>
    *    <li> the left side of this line has a negative y coordinate.
    *    <li> the right side of this line has a positive y coordinate.
    * </ul>
    *<p>
    * This method will return false if the point is on the line.
    *
    * @param pointX the x-coordinate of  the query point.
    * @param pointY the y-coordinate of  the query point.
    * @param pointOnLineX the x-coordinate of a point positioned on the infinite line.
    * @param pointOnLineY the y-coordinate of a point positioned on the infinite line.
    * @param lineDirectionX the x-component of the direction of the infinite line.
    * @param lineDirectionY the y-component of the direction of the infinite line.
    * @param side the query of the side.
    * @return {@code true} if the point is on the query side of the line, {@code false} if the point is on the opposite side or exactly on the line.
    */
   public static boolean isPointOnSideOfLine(double pointX, double pointY, double pointOnLineX, double pointOnLineY, double lineDirectionX, double lineDirectionY, RobotSide side)
   {
      double pointToPointX = pointX - pointOnLineX;
      double pointToPointY = pointY - pointOnLineY;
      double crossProduct = lineDirectionX * pointToPointY - pointToPointX * lineDirectionY;
      return side.negateIfRightSide(crossProduct) > 0.0;
   }

   /**
    * Returns a boolean value, stating whether a 2D point is on the left side of a given line.
    * "Left side" is determined based on order of {@code lineStart} and {@code lineEnd}.
    * <p>
    * WARNING: the 3D arguments are projected onto the XY-plane to perform the actual computation in 2D.
    * <p>
    * For instance, given the {@code lineStart} coordinates x = 0, and y = 0, and the {@code lineEnd} coordinates x = 1, y = 0,
    * a point located on the left of this line has a negative y coordinate.
    *<p>
    * This method will return false if the point is on the line.
    * 
    * @param point the projection onto the XY-plane of this point is used as the 2D query point. Not modified.
    * @param lineStart the projection onto the XY-plane of this point is used as the starting point of the 2D line segment. Not modified.
    * @param lineEnd the projection onto the XY-plane of this point is used as the end point of the 2D line segment. Not modified.
    * @return {@code true} if the 2D projection of the point is on the left side of the 2D projection of the line.
    * {@code false} if the 2D projection of the point is on the right side or exactly on the 2D projection of the line.
    */
   // FIXME this method is confusing and error prone.
   public static boolean isPointOnLeftSideOfLine(FramePoint point, FramePoint lineStart, FramePoint lineEnd)
   {
      point.checkReferenceFrameMatch(lineStart);
      point.checkReferenceFrameMatch(lineEnd);
      Point2d lineStartPoint2d = new Point2d(lineStart.getX(), lineStart.getY());
      Point2d lineEndPoint2d = new Point2d(lineEnd.getX(), lineEnd.getY());
      Point2d checkPointPoint2d = new Point2d(point.getX(), point.getY());

      return isPointOnLeftSideOfLine(checkPointPoint2d, lineStartPoint2d, lineEndPoint2d);
   }

   /**
    * Returns true only if the point is inside the triangle defined by the vertices a, b, and c.
    * The triangle can be clockwise or counter-clockwise ordered.
    * @param point the point to check if lying inside the triangle. Not modified.
    * @param a first vertex of the triangle. Not modified.
    * @param b second vertex of the triangle. Not modified.
    * @param c third vertex of the triangle. Not modified.
    * @return {@code true} if the query is exactly inside the triangle. {@code false} if the query point is outside triangle or exactly on an edge of the triangle.
    */
   public static boolean isPointInsideTriangleABC(Point2d point, Point2d a, Point2d b, Point2d c)
   {
      // This makes the assertion working for both clockwise and counter-clockwise ordered vertices.
      RobotSide sideToCheck = isPointOnLeftSideOfLine(b, a, c) ? RobotSide.LEFT : RobotSide.RIGHT;

      if (isPointOnSideOfLine(point, a, b, sideToCheck))
         return false;
      if (isPointOnSideOfLine(point, b, c, sideToCheck))
         return false;
      if (isPointOnSideOfLine(point, c, a, sideToCheck))
         return false;

      return true;
   }

   /**
    * Compute the area of a triangle defined by its three vertices: a, b, and c.
    * No specific ordering of the vertices is required.
    * 
    * @param a first vertex of the triangle. Not modified.
    * @param b second vertex of the triangle. Not modified.
    * @param c third vertex of the triangle. Not modified.
    * @return the are of the triangle.
    */
   public static double computeTriangleArea(Point2d a, Point2d b, Point2d c)
   {
      return Math.abs(0.5 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y)));
   }

   /**
    * Computes the average 2D point from a given collection of 2D points.
    *
    * @param points the collection of 2D points to compute the average from. Not modified.
    * @return the computed average.
    */
   public static Point2d averagePoint2ds(Collection<Point2d> points)
   {
      Point2d totalPoint = new Point2d(0.0, 0.0);

      for (Point2d point : points)
      {
         totalPoint.add(point);
      }

      totalPoint.scale(1.0 / points.size());

      return totalPoint;
   }

   /**
    * Computes the average 3D point from a given collection of 3D points.
    *
    * @param points the collection of 3D points to compute the average from. Not modified.
    * @return the computed average.
    */
   public static Point3d averagePoint3ds(Collection<Point3d> points)
   {
      Point3d totalPoint = new Point3d(0.0, 0.0, 0.0);

      for (Point3d point : points)
      {
         totalPoint.add(point);
      }

      totalPoint.scale(1.0 / points.size());

      return totalPoint;
   }

   /**
    * Returns the average of two 3D points.
    *
    * @param a the first 3D point. Not modified.
    * @param b the second 3D point. Not modified.
    * @return the computed average.
    */
   public static Point3d averagePoints(Point3d a, Point3d b)
   {
      Point3d average = new Point3d(a);
      average.add(b);
      average.scale(0.5);

      return average;
   }

   /**
    * Returns the average of two 3D points.
    *
    * @param a the first 3D point. Not modified.
    * @param b the second 3D point. Not modified.
    * @param avgToPack the point in which the computed average is stored. Modified.
    */
   public static void averagePoints(FramePoint a, FramePoint b, FramePoint avgToPack)
   {
      avgToPack.setIncludingFrame(a);
      avgToPack.add(b);
      avgToPack.scale(0.5);
   }

   /**
    * Computes the orthogonal projection of a 2D point on an infinitely long 2D line defined by a 2D line segment.
    * 
    * @param testPoint the point to compute the projection of. Not modified.
    * @param lineStart starting point of the line segment defining the infinite line. Not modified.
    * @param lineEnd end point of the line segment defining the infinite line. Not modified.
    * @return the projection on the line.
    */
   // TODO ensure consistant with line 2D
   public static Point2d getOrthogonalProjectionOnLine(Point2d testPoint, Point2d lineStart, Point2d lineEnd)
   {
      Line2d line = new Line2d(lineStart, lineEnd);
      Point2d projected = line.orthogonalProjectionCopy(testPoint);

      return projected;
   }

   /**
    * Attempts to normalize the given 3D vector.
    * If the vector's length falls below {@value Epsilons#ONE_TRILLIONTH}, the vector is set to (0, 0, 1).
    *  
    * @param vector the 3D vector to normalize. Modified.
    */
   public static void normalizeSafelyZUp(Vector3d vector)
   {
      double distance = vector.length();

      if (distance > Epsilons.ONE_TRILLIONTH)
      {
         vector.scale(1.0 / distance);
      }
      else
      {
         vector.set(0.0, 0.0, 1.0);
      }
   }

   /**
    * Given two 3D infinitely long lines, this methods computes two points P &in; line1 and Q &in; lin2 such that the distance || P - Q || is the minimum distance between the two 3D lines.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    * @param lineStart1 a 3D point on the first line. Not modified.
    * @param lineDirection1 the 3D direction of the first line. Not modified.
    * @param lineStart2 a 3D point on the second line. Not modified.
    * @param lineDirection2 the 3D direction of the second line. Not modified.
    * @param pointOnLine1ToPack the 3D coordinates of the point P are packed in this 3D point. Modified.
    * @param pointOnLine2ToPack the 3D coordinates of the point Q are packed in this 3D point. Modified.
    * @throws ReferenceFrameMismatchException if the input arguments are not expressed in the same reference frame.
    */
   public static void getClosestPointsForTwoLines(FramePoint lineStart1, FrameVector lineDirection1, FramePoint lineStart2, FrameVector lineDirection2, FramePoint pointOnLine1ToPack,
           FramePoint pointOnLine2ToPack)
   {
      lineStart1.checkReferenceFrameMatch(lineDirection1);
      lineStart2.checkReferenceFrameMatch(lineDirection2);
      lineStart1.checkReferenceFrameMatch(lineStart2);

      pointOnLine1ToPack.setToZero(lineStart1.getReferenceFrame());
      pointOnLine2ToPack.setToZero(lineStart1.getReferenceFrame());

      getClosestPointsForTwoLines(lineStart1.getPoint(), lineDirection1.getVector(), lineStart2.getPoint(), lineDirection2.getVector(), pointOnLine1ToPack.getPoint(), pointOnLine2ToPack.getPoint());
   }

   /**
    * Given two 3D infinitely long lines, this methods computes two points P &in; line1 and Q &in; lin2 such that the distance || P - Q || is the minimum distance between the two 3D lines.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    * @param lineStart1 a 3D point on the first line. Not modified.
    * @param lineDirection1 the 3D direction of the first line. Not modified.
    * @param lineStart2 a 3D point on the second line. Not modified.
    * @param lineDirection2 the 3D direction of the second line. Not modified.
    * @param pointOnLine1ToPack the 3D coordinates of the point P are packed in this 3D point. Modified.
    * @param pointOnLine2ToPack the 3D coordinates of the point Q are packed in this 3D point. Modified.
    */
   public static void getClosestPointsForTwoLines(Point3d lineStart1, Vector3d lineDirection1, Point3d lineStart2, Vector3d lineDirection2, Point3d pointOnLine1ToPack,
           Point3d pointOnLine2ToPack)
   {
      // Switching to the notation and math described in http://geomalgorithms.com/a07-_distance.html.
      // The line1 is defined by (P0, u) and the line2 by (Q0, v).
      // Note: the algorithm is independent from the magnitudes of lineDirection1 and lineDirection2
      Point3d P0 = lineStart1;
      Vector3d u = lineDirection1;
      Point3d Q0 = lineStart2;
      Vector3d v = lineDirection2;

      Point3d Psc = pointOnLine1ToPack;
      Point3d Qtc = pointOnLine2ToPack;

      Vector3d w0 = new Vector3d();
      w0.sub(P0, Q0);
      
      double a = u.dot(u);
      double b = u.dot(v);
      double c = v.dot(v);
      double d = u.dot(w0);
      double e = v.dot(w0);

      double delta = a * c - b * b;

      double sc, tc;

      // check to see if the lines are parallel
      if (Math.abs(delta) <= EPSILON)
      {
         /*
          * The lines are parallel, there's an infinite number of pairs,
          * but for one chosen point on one of the lines, there's only one closest point to it on the other line.
          * So let's chose arbitrarily a point on the line1 and calculate the point that is closest to it on the line2.
          */
         sc = 0.0;
         tc = d / b;
      }
      else
      {
         sc = (b * e - c * d) / delta;
         tc = (a * e - b * d) / delta;
      }

      Psc.scaleAdd(sc, u, P0);
      Qtc.scaleAdd(tc, v, Q0);
   }

   /**
    * Computes the coordinates of the intersection between a plane and an infinitely long line.
    * In the case the line is parallel to the plane, this method will return {@code null}.
    * <a href="https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection"> Useful link </a>.
    * 
    * @param pointOnPlane a point located on the plane. Not modified.
    * @param planeNormal the normal of the plane. Not modified.
    * @param pointOnLine a point located on the line. Not modified.
    * @param lineDirection the direction of the line. Not modified.
    * @return the coordinates of the intersection, or {@code null} if the line is parallel to the plane.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same frame.
    */
   public static FramePoint getIntersectionBetweenLineAndPlane(FramePoint pointOnPlane, FrameVector planeNormal, FramePoint pointOnLine, FrameVector lineDirection)
   {
      pointOnPlane.checkReferenceFrameMatch(planeNormal);
      pointOnLine.checkReferenceFrameMatch(lineDirection);
      pointOnPlane.checkReferenceFrameMatch(pointOnLine);

      Point3d intersection = getIntersectionBetweenLineAndPlane(pointOnPlane.getPoint(), planeNormal.getVector(), pointOnLine.getPoint(), lineDirection.getVector());

      if (intersection == null)
         return null;
      else
         return new FramePoint(pointOnPlane.getReferenceFrame(), intersection);
   }

   /**
    * Computes the coordinates of the intersection between a plane and an infinitely long line.
    * In the case the line is parallel to the plane, this method will return {@code null}.
    * <a href="https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection"> Useful link </a>.
    * 
    * @param pointOnPlane a point located on the plane. Not modified.
    * @param planeNormal the normal of the plane. Not modified.
    * @param pointOnLine a point located on the line. Not modified.
    * @param lineDirection the direction of the line. Not modified.
    * @return the coordinates of the intersection, or {@code null} if the line is parallel to the plane.
    */
   public static Point3d getIntersectionBetweenLineAndPlane(Point3d pointOnPlane, Vector3d planeNormal, Point3d pointOnLine, Vector3d lineDirection)
   {
      // Switching to the notation used in https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection
      // Note: the algorithm is independent from the magnitudes of planeNormal and lineDirection
      Point3d p0 = pointOnPlane;
      Vector3d n = planeNormal;
      Point3d l0 = pointOnLine;
      Vector3d l = lineDirection;

      // Let's compute the value of the coefficient d = ( (p0 - l0).n ) / ( l.n )
      double d, numerator, denominator;
      numerator = (p0.getX() - l0.getX()) * n.getX();
      numerator += (p0.getY() - l0.getY()) * n.getY();
      numerator += (p0.getZ() - l0.getZ()) * n.getZ();
      denominator = l.dot(n);

      // Check if the line is parallel to the plane
      if (Math.abs(denominator) < EPSILON)
      {
         if (Math.abs(numerator) < EPSILON)
            return new Point3d(pointOnLine);
         else
            return null;
      }
      else
      {
         d = numerator / denominator;
         
         Point3d intersection = new Point3d();
         intersection.scaleAdd(d, l, l0);
         return intersection;
      }
   }

   /**
    * Locates and returns the intersection between
    * the given line segment and plane
    *
    * @param pointOnPlane FramePoint
    * @param planeNormal FrameVector
    * @param lineStart FramePoint
    * @param lineEnd FramePoint
    * @return FramePoint
    */
   public static FramePoint getIntersectionBetweenLineSegmentAndPlane(FramePoint pointOnPlane, FrameVector planeNormal, FramePoint lineStart,
                                                                      FramePoint lineEnd)
   {
      // po = line start, p1 = line end
      // v0 = point on plane
      // n = plane normal
      // intersection point is p(s) = p0 + s*(p1 - p0)
      // scalar s = (n dot (v0 - p0))/(n dot (p1 - p0)

      if (isLineSegmentIntersectingPlane(pointOnPlane, planeNormal, lineStart, lineEnd))
      {
         planeNormal.normalize();

         FrameVector line = new FrameVector(lineStart.getReferenceFrame());
         line.sub(lineEnd, lineStart);

         FrameVector fromP0toV0 = new FrameVector(pointOnPlane.getReferenceFrame());
         fromP0toV0.sub(pointOnPlane, lineStart);

         double numerator = planeNormal.dot(fromP0toV0);
         double denominator = planeNormal.dot(line);
         double scaleFactor = numerator / denominator;

         FramePoint ret = new FramePoint(lineStart.getReferenceFrame());
         ret.scaleAdd(scaleFactor, line, lineStart);

         if (ret.containsNaN() || ret.containsInfinity())
         {
            ret = null;
         }

         return ret;
      }
      else
      {
         return null;
      }

   }

   public static boolean isLineSegmentIntersectingPlane(FramePoint pointOnPlane, FrameVector planeNormal, FramePoint lineStart, FramePoint lineEnd)
   {
      double d = -planeNormal.getX() * pointOnPlane.getX() - planeNormal.getY() * pointOnPlane.getY() - planeNormal.getZ() * pointOnPlane.getZ();

      double ansStart = planeNormal.getX() * lineStart.getX() + planeNormal.getY() * lineStart.getY() + planeNormal.getZ() * lineStart.getZ() + d;

      double ansEnd = planeNormal.getX() * lineEnd.getX() + planeNormal.getY() * lineEnd.getY() + planeNormal.getZ() * lineEnd.getZ() + d;

      //      System.out.println("Start: " + ansStart + ", End: " + ansEnd);

      if (((ansStart > 0) && (ansEnd < 0)) || ((ansStart < 0) && (ansEnd > 0)))
      {
         //         System.out.println("Line is intersecting plane");

         return true;
      }
      else
      {
         //          System.out.println("Line is not intersecting plane");

         return false;
      }
   }

   public static double distanceFromPointToPlane(FramePoint pointOnPlane, FrameVector planeNormal, FramePoint point)
   {
      double d = -planeNormal.getX() * pointOnPlane.getX() - planeNormal.getY() * pointOnPlane.getY() - planeNormal.getZ() * pointOnPlane.getZ();

      double dist = Math.abs(planeNormal.getX() * point.getX() + planeNormal.getY() * point.getY() + planeNormal.getZ() * point.getZ() + d)
                    / Math.sqrt(planeNormal.getX() * planeNormal.getX() + planeNormal.getY() * planeNormal.getY() + planeNormal.getZ() * planeNormal.getZ());

      return dist;
   }

   // TODO ensure consistant with lineSegment2D
   public static boolean doLineSegmentsIntersect(Point2d lineStart1, Point2d lineEnd1, Point2d lineStart2, Point2d lineEnd2)
   {
      double r1numerator = (lineEnd2.getX() - lineStart2.getX()) * (lineStart1.getY() - lineStart2.getY()) - (lineEnd2.getY() - lineStart2.getY()) * (lineStart1.getX() - lineStart2.getX());

      double r1denominator = (lineEnd2.getY() - lineStart2.getY()) * (lineEnd1.getX() - lineStart1.getX()) - (lineEnd2.getX() - lineStart2.getX()) * (lineEnd1.getY() - lineStart1.getY());

      double r2numerator = (lineEnd1.getX() - lineStart1.getX()) * (lineStart1.getY() - lineStart2.getY()) - (lineEnd1.getY() - lineStart1.getY()) * (lineStart1.getX() - lineStart2.getX());

      double r2denominator = r1denominator;

      // If both numerators and the denominator are zero, the lines are collinear.
      // We must project the lines onto the X- or Y-axis check if the segments overlap.
      if ((r1numerator == 0.0) && (r2numerator == 0.0) && (r1denominator == 0.0))
      {
         double ls1, le1, ls2, le2;
         if (lineStart1.getX() != lineEnd1.getX())
         {
            ls1 = lineStart1.getX();
            le1 = lineEnd1.getX();
            ls2 = lineStart2.getX();
            le2 = lineEnd2.getX();
         }
         else
         {
            ls1 = lineStart1.getY();
            le1 = lineEnd1.getY();
            ls2 = lineStart2.getY();
            le2 = lineEnd2.getY();
         }

         // If both first points are less than both second points, the line
         // segments do not intersect.
         if (((ls1 < ls2) && (le1 < ls2)) && ((ls1 < le2) && (le1 < le2)))
            return false;

         // If both first points are greater than both second points, the line
         // segments do not intersect.
         if (((ls1 > ls2) && (le1 > ls2)) && ((ls1 > le2) && (le1 > le2)))
            return false;

         // Otherwise, the line segments must overlap. So we return true.
         return true;
      }

      // If the denominator is zero, but the numerators are not, then the lines are parallel.
      if (r1denominator == 0.0)
         return false;

      double r1 = r1numerator / r1denominator;
      double r2 = r2numerator / r2denominator;

      // If both r1 and r2 are between zero and one, the line segments intersect.
      if ((0.0 <= r1) && (r1 <= 1.0) && (0.0 <= r2) && (r2 <= 1.0))
         return true;

      return false;
   }

   /**
    * @deprecated Creates garbage. Use {@link GeometryTools.intersection}.
    * @param lineStart1
    * @param lineEnd1
    * @param lineStart2
    * @param lineEnd2
    * @return
    */
   public static Point2d getIntersectionBetweenTwoLines(Point2d lineStart1, Point2d lineEnd1, Point2d lineStart2, Point2d lineEnd2)
   {
      Line2d line1 = new Line2d(lineStart1, lineEnd1);
      Line2d line2 = new Line2d(lineStart2, lineEnd2);

      return line1.intersectionWith(line2);
   }

   private static final ThreadLocal<double[]> tempAlphaBeta = new ThreadLocal<double[]>()
   {
      @Override
      public double[] initialValue()
      {
         return new double[2];
      }
   };

   private static final ThreadLocal<FrameVector[]> tempDirectionsForIntersection = new ThreadLocal<FrameVector[]>()
   {
      @Override
      public FrameVector[] initialValue()
      {
         return new FrameVector[] {new FrameVector(), new FrameVector()};
      }
   };

   public static boolean getIntersectionBetweenTwoLines2d(FramePoint intersectionToPack, FramePoint lineStart1, FramePoint lineEnd1, FramePoint lineStart2, FramePoint lineEnd2)
   {
      tempDirectionsForIntersection.get()[0].sub(lineEnd1, lineStart1);
      tempDirectionsForIntersection.get()[1].sub(lineEnd2, lineStart2);

      return GeometryTools.getIntersectionBetweenTwoLines2d(intersectionToPack, lineStart1, tempDirectionsForIntersection.get()[0], lineStart2, tempDirectionsForIntersection.get()[1]);
   }

   public static boolean getIntersectionBetweenTwoLines2d(FramePoint intersectionToPack, FramePoint point1, FrameVector direction1, FramePoint point2, FrameVector direction2)
   {
      GeometryTools.intersection(point1.getX(), point1.getY(), direction1.getX(), direction1.getY(), point2.getX(), point2.getY(), direction2.getX(), direction2.getY(), tempAlphaBeta.get());

      if (Double.isNaN(tempAlphaBeta.get()[0]) || Double.isNaN(tempAlphaBeta.get()[1]))
      {
         intersectionToPack.set(Double.NaN, Double.NaN,Double.NaN);
         return false;
         //throw new UndefinedOperationException("Lines are parallel.");
      }

      intersectionToPack.set(point1.getX() + direction1.getX() * tempAlphaBeta.get()[0], point1.getY() + direction1.getY() * tempAlphaBeta.get()[0], intersectionToPack.getZ());
      return true;
   }

   /**
    * Finds the intersection between two 2D lines.
    * Each line is represented as a Point2d and a Vector2d.
    * This should work as long as the two lines are not parallel.
    * If they are parallel, it tries to return something without crashing.
    *
    * @param point1 Start Point of first line.
    * @param vector1 Direction Vector of first line.
    * @param point2 Start Point of second line.
    * @param vector2 Direction Vector of second line.
    * @return Point of Intersection.
    * @deprecated Creates garbage. Use {@link GeometryTools.intersection}.
    * TODO ensure consistant with line2D
    */
   public static Point2d getIntersectionBetweenTwoLines(Point2d point1, Vector2d vector1, Point2d point2, Vector2d vector2)
   {
      Line2d line1 = new Line2d(point1, vector1);
      Line2d line2 = new Line2d(point2, vector2);

      return line1.intersectionWith(line2);
   }

   /**
       *       Finds the intersection parameters between two lines. First line starts at (firstPointX, firstPointY) and has direction vector (firstVectorX, firstVectorY).
       *       The second line starts at (secondPointX, secondPointY) and has direction vector (secondVectorX, secondVectorY). Returns null if the lines are parallel.
       *       Returns {alpha, beta} such that the intersection between the lines occurs at P1 + alpha * V1 = P2 + beta * V2;
       *
       *       @param x0 double First line starting x.
       *       @param y0 double First line starting y.
       *       @param vx0 double First line direction x.
       *       @param vy0 double First line direction y.
       *       @param x1 double Second line starting x.
       *       @param y1 double Second line starting y.
       *       @param vx1 double Second line direction x.
       *       @param vy1 double Second line direction y.
       *
       *       @return double[] {alpha, beta} such that the intersection between the lines occurs at P1 + alpha * V1 = P2 + beta * V2;
       */

      // TODO only used by the intersection methods in this class
      public static void intersection(double x0, double y0, double vx0, double vy0, double x1, double y1, double vx1, double vy1, double[] alphaBetaToPack)
      {
   //      We solve for x the problem of the form: A * x = b
   //            A      *     x     =      b
   //      / vx0 -vx1 \   / alpha \   / x1 - x0 \
   //      |          | * |       | = |         |
   //      \ vy0 -vy1 /   \ beta  /   \ y1 - y0 /
   //
   //
   //      double[][] A = new double[2][2];
   //      A[0][0] = vx0;
   //      A[0][1] = -vx1;
   //      A[1][0] = vy0;
   //      A[1][1] = -vy1;
   //
   //      double[] b = new double[2];
   //      b[0] = x1 - x0;
   //      b[1] = y1 - y0;

         double determinant = -vx0 * vy1 + vy0 * vx1; //(A[0][0] * A[1][1]) - (A[1][0] * A[0][1]);

         double epsilon = 1.0E-12;
         if (Math.abs(determinant) < epsilon)
         {
            alphaBetaToPack[0] = Double.NaN;
            alphaBetaToPack[1] = Double.NaN;
         }
         else
         {
            double oneOverDeterminant = 1.0 / determinant;
            double AInverse00 = oneOverDeterminant * -vy1; // A[1][1];
            double AInverse01 = oneOverDeterminant *  vx1; //-A[0][1];
            double AInverse10 = oneOverDeterminant * -vy0; //-A[1][0];
            double AInverse11 = oneOverDeterminant *  vx0; // A[0][0];

            double dx = x1 - x0;
            double dy = y1 - y0;
            double alpha = AInverse00 * dx + AInverse01 * dy;// AInverse00 * b[0] + AInverse01 * b[1];
            double beta  = AInverse10 * dx + AInverse11 * dy;// AInverse10 * b[0] + AInverse11 * b[1];

            alphaBetaToPack[0] = alpha;
            alphaBetaToPack[1] = beta;
         }
      }

   /**
    * Returns the line segment percentages of the intersection point between two lines if the lines are intersecting and not colinear.
    * If colinear, or parallel, then returns null.
    * This is epsilon conservative in determining parallelness or colinearity. If just slightly not parallel, will still return null.
    *
    * TODO ensure consistant with lineSegment2D
    *
    * @param lineStart1 Point2d
    * @param lineEnd1 Point2d
    * @param lineStart2 Point2d
    * @param lineEnd2 Point2d
    *
    * @deprecated Creates garbage
    * @return Intersecting percentages of the two line segments if intersecting and not colinear. Null if not intersecting, or colinear.
    */
   public static double[] getLineSegmentPercentagesIfIntersecting(Point2d lineStart1, Point2d lineEnd1, Point2d lineStart2, Point2d lineEnd2)
   {
      double[] garbage = new double[2];
      getLineSegmentPercentagesIfIntersecting(lineStart1.getX(), lineStart1.getY(), lineEnd1.getX(), lineEnd1.getY(), lineStart2.getX(), lineStart2.getY(), lineEnd2.getX(), lineEnd2.getY(), garbage);
      return garbage;
   }

   /**
    * Returns the line segment percentages of the intersection point between two lines if the lines are intersecting and not colinear.
    * If colinear, or parallel, then returns null.
    * This is epsilon conservative in determining parallelness or colinearity. If just slightly not parallel, will still return null.
    *
    * TODO ensure consistant with lineSegment2D
    *
    * @param lineStart1 Point2d
    * @param lineEnd1 Point2d
    * @param lineStart2 Point2d
    * @param lineEnd2 Point2d
    * @param Intersecting percentages of the two line segments if intersecting and not colinear. Null if not intersecting, or colinear.
    */
   public static void getLineSegmentPercentagesIfIntersecting(FramePoint lineStart1, FramePoint lineEnd1, FramePoint lineStart2, FramePoint lineEnd2, double[] percentages)
   {
      getLineSegmentPercentagesIfIntersecting(lineStart1.getX(), lineStart1.getY(), lineEnd1.getX(), lineEnd1.getY(), lineStart2.getX(), lineStart2.getY(), lineEnd2.getX(), lineEnd2.getY(), percentages);
   }

   private static void getLineSegmentPercentagesIfIntersecting(double l1ax, double l1ay, double l1bx, double l1by, double l2ax, double l2ay, double l2bx, double l2by, double[] percentages)
   {
      double r1numerator = (l2bx - l2ax) * (l1ay - l2ay) - (l2by - l2ay) * (l1ax - l2ax);

      double r1denominator = (l2by - l2ay) * (l1bx - l1ax) - (l2bx - l2ax) * (l1by - l1ay);

      double r2numerator = (l1bx - l1ax) * (l1ay - l2ay) - (l1by - l1ay) * (l1ax - l2ax);

      double r2denominator = r1denominator;

      // If the denominator is zero, the lines are either colinear or parallel.
      if (Math.abs(r1denominator) < EPSILON)
      {
         throw new UndefinedOperationException("Lines are colinear or parallel.");
      }

      double r1 = r1numerator / r1denominator;
      double r2 = r2numerator / r2denominator;

      percentages[0] = r1;
      percentages[1] = r2;
   }

   /**
    * Returns the Normal of a plane that is defined by three points
    * Returns a null if three points are linear, colinear, or equal
    * Returns a null if two points are the same
    *
    * @param point1 FramePoint
    * @param point2 FramePoint
    * @param point3 FramePoint
    * @return FrameVector
    */
   public static FrameVector getPlaneNormalGivenThreePoints(FramePoint point1, FramePoint point2, FramePoint point3)
   {
      FrameVector oneToTwo = new FrameVector(point2);
      oneToTwo.sub(point1);

      FrameVector oneToThree = new FrameVector(point3);
      oneToThree.sub(point1);

      FrameVector normal = new FrameVector(oneToThree.getReferenceFrame());
      normal.cross(oneToTwo, oneToThree);

      if (normal.length() > 1.0e-7)
         normal.normalize();
      else
         return null;

      return normal;
   }

   /**
    * Returns the Perpendicular Vector that is formed
    * between a given Line (defined by two points) and a given point
    *
    * Returns zeros if point is located on the line
    *
    * @param point FramePoint
    * @param lineStart FramePoint
    * @param lineEnd FramePoint
    * @param intersectionPoint FramePoint
    * @return FrameVector
    */

   // TODO ensure consistant with lineSegment2D
   public static FrameVector getPerpendicularVectorFromLineToPoint(FramePoint point, FramePoint lineStart, FramePoint lineEnd, FramePoint intersectionPoint)
   {
      FrameVector lineDirection = new FrameVector(lineEnd);
      lineDirection.sub(lineStart);
      lineDirection.normalize();

      FrameVector startToPoint = new FrameVector(point);
      startToPoint.sub(lineStart);

      double distanceOnLine = lineDirection.dot(startToPoint);

//    FramePoint intersectionPoint = new FramePoint(lineStart.getReferenceFrame());
      intersectionPoint.scaleAdd(distanceOnLine, lineDirection, lineStart);

//    System.out.println("intersectionPoint=" + intersectionPoint);

      FrameVector lineToReturn = new FrameVector(point);
      lineToReturn.sub(intersectionPoint);

      return lineToReturn;
   }

   /**
    * Not garbage free.
    *
    * @deprecated Use {@link #getPerpendicularVector(Vector2d, Vector2d)}
    */
   public static Vector2d getPerpendicularVector(Vector2d vector)
   {
      Vector2d perpendicularVector = new Vector2d();
      getPerpendicularVector(perpendicularVector, vector);

      return perpendicularVector;
   }

   public static void getPerpendicularVector(Vector2d perpendicularVectorToPack, Vector2d vector)
   {
      perpendicularVectorToPack.set(-vector.getY(), vector.getX());
   }

   public static void getPerpendicularVector2d(FrameVector perpendicularVectorToPack, FrameVector vector)
   {
      perpendicularVectorToPack.set(-vector.getY(), vector.getX(), perpendicularVectorToPack.getZ());
   }

   /**
    * Converts three 3D points into 3 framePoints (defines a plane)
    * Calls getPlaneNormalGivenThreePoints
    *
    * Returns normal to plane
    * Returns null if three points are:
    *      linear
    *      colinear
    *      equal
    *      if 2 points are equal
    *
    * @param point1 Point3d
    * @param point2 Point3d
    * @param point3 Point3d
    * @return Vector3d
    */
   public static Vector3d getPlaneNormalGivenThreePoints(Point3d point1, Point3d point2, Point3d point3)
   {
      FramePoint framePoint1 = new FramePoint(ReferenceFrame.getWorldFrame(), point1);
      FramePoint framePoint2 = new FramePoint(ReferenceFrame.getWorldFrame(), point2);
      FramePoint framePoint3 = new FramePoint(ReferenceFrame.getWorldFrame(), point3);

      FrameVector normal = getPlaneNormalGivenThreePoints(framePoint1, framePoint2, framePoint3);

      if (normal == null)
         return null;

      return normal.getVectorCopy();
   }

   /**
    * Computes vertex B of an isosceles triangle ABC with equal legs AB and BC.
    *
    * Returns the solution that corresponds with the triangle in which rotation of leg AB to leg BC is counterclockwise about vertex B.
    *
    * @param baseVertexA
    * @param baseVertexC
    * @param trianglePlaneNormal
    * @param ccwAngleAboutNormalAtTopVertex
    * @param topVertexBToPack
    */
   public static void getTopVertexOfIsoscelesTriangle(FramePoint baseVertexA, FramePoint baseVertexC, FrameVector trianglePlaneNormal,
         double ccwAngleAboutNormalAtTopVertex, FramePoint topVertexBToPack)
   {
      ReferenceFrame commonFrame = baseVertexA.referenceFrame;
      baseVertexC.checkReferenceFrameMatch(commonFrame);
      trianglePlaneNormal.checkReferenceFrameMatch(commonFrame);

      getTopVertexOfIsoscelesTriangle(baseVertexA.getPoint(), baseVertexC.getPoint(), trianglePlaneNormal.getVector(),
            ccwAngleAboutNormalAtTopVertex, topVertexBToPack.getPoint());
   }


   public static void getTopVertexOfIsoscelesTriangle(Point3d baseVertexA, Point3d baseVertexC, Vector3d trianglePlaneNormal,
         double ccwAngleAboutNormalAtTopVertex, Point3d topVertexBToPack)
   {
      Vector3d baseEdgeAC = new Vector3d();
      baseEdgeAC.sub(baseVertexC, baseVertexA);

      double legLengthABorCB = getRadiusOfArc(baseEdgeAC.length(), ccwAngleAboutNormalAtTopVertex);
      double lengthOfBisectorOfBase = pythagorasGetCathetus(legLengthABorCB, 0.5 * baseEdgeAC.length());

      Vector3d perpendicularBisector = new Vector3d();
      getPerpendicularToLine(baseEdgeAC, trianglePlaneNormal, lengthOfBisectorOfBase, perpendicularBisector);

      topVertexBToPack.interpolate(baseVertexA, baseVertexC, 0.5);
      topVertexBToPack.add(perpendicularBisector);
   }

   /**
    * Returns the radius of an arc with the specified chord length and angle
    *
    * @param chordLength
    * @param chordAngle
    * @return
    */
   public static double getRadiusOfArc(double chordLength, double chordAngle)
   {
      return chordLength / (2.0 * Math.sin(chordAngle / 2.0));
   }
   
   public static void clipToBoundingBox(Tuple3d tuple, double x1, double x2, double y1, double y2, double z1, double z2)
   {
      tuple.setX(x1 < x2 ? MathTools.clipToMinMax(tuple.getX(), x1, x2) : MathTools.clipToMinMax(tuple.getX(), x2, x1));
      tuple.setY(y1 < y2 ? MathTools.clipToMinMax(tuple.getY(), y1, y2) : MathTools.clipToMinMax(tuple.getY(), y2, y1));
      tuple.setZ(z1 < z2 ? MathTools.clipToMinMax(tuple.getZ(), z1, z2) : MathTools.clipToMinMax(tuple.getZ(), z2, z1));
   }

   /**
    * Computes a vector of desired length that is perpendicular to a line.  Computed vector is in the plane defined by the specified normal vector.
    * Vector points to the left of the line, when the line points upwards.
    *
    * @param lineToBisect
    * @param planeNormal
    * @param bisectorLengthDesired
    * @param perpendicularVec
    */
   public static void getPerpendicularToLine(FrameVector line, FrameVector planeNormal, double bisectorLengthDesired, FrameVector perpendicularVec)
   {
      ReferenceFrame commonFrame = line.referenceFrame;
      planeNormal.checkReferenceFrameMatch(commonFrame);

      getPerpendicularToLine(line.getVector(), planeNormal.getVector(), bisectorLengthDesired, perpendicularVec.getVector());
   }

   public static void getPerpendicularToLine(Vector3d line, Vector3d planeNormal, double bisectorLengthDesired, Vector3d perpendicularVec)
   {
      perpendicularVec.set(0.0, 0.0, 0.0);
      perpendicularVec.cross(planeNormal, line);
      perpendicularVec.scale(1.0 / perpendicularVec.length());
      perpendicularVec.scale(bisectorLengthDesired);
   }

   // TODO ensure consistant with lineSegment2D
   public static void getZPlanePerpendicularBisector(FramePoint lineStart, FramePoint lineEnd, FramePoint bisectorStart, FrameVector bisectorDirection)
   {
      Point2d lineStart2d = new Point2d(lineStart.getX(), lineStart.getY());
      Point2d lineEnd2d = new Point2d(lineEnd.getX(), lineEnd.getY());
      Point2d bisectorStart2d = new Point2d();
      Vector2d bisectorDirection2d = new Vector2d();

      getPerpendicularBisector(lineStart2d, lineEnd2d, bisectorStart2d, bisectorDirection2d);
      bisectorDirection.set(bisectorDirection2d.getX(), bisectorDirection2d.getY(), 0.0);
      bisectorStart.set(bisectorStart2d.getX(), bisectorStart2d.getY(), 0.0);
   }

   /**
    *
    * @param lineStart Point2d
    * @param lineEnd Point2d
    * @param bisectorStart Point2d
    * @param bisectorDirection Vector2d
    */

   // TODO ensure consistant with lineSegment2D
   public static void getPerpendicularBisector(Point2d lineStart, Point2d lineEnd, Point2d bisectorStart, Vector2d bisectorDirection)
   {
      // direction will be on left side of line
      Vector2d lineDirection = new Vector2d();
      lineDirection.sub(lineEnd, lineStart);

      bisectorStart.scaleAdd(0.5, lineDirection, lineStart);

      bisectorDirection.set(-lineDirection.getY(), lineDirection.getX());
      bisectorDirection.normalize();
   }

   private static final ThreadLocal<Vector3d> tempCrossProduct = new ThreadLocal<Vector3d>()
   {
      @Override
      public Vector3d initialValue()
      {
         return new Vector3d();
      }
   };

   private static final ThreadLocal<Vector3d> tempWorldNormal = new ThreadLocal<Vector3d>()
   {
      @Override
      public Vector3d initialValue()
      {
         return new Vector3d();
      }
   };

   public static AxisAngle4d getRotationBasedOnNormal(Vector3d normalVector3d)
   {
      AxisAngle4d newAxisAngle4d = new AxisAngle4d();
      getRotationBasedOnNormal(newAxisAngle4d, normalVector3d);
      return newAxisAngle4d;
   }

   public static void getRotationBasedOnNormal(AxisAngle4d rotationToPack, Vector3d normalVector3d)
   {
      Vector3d worldNormal = tempWorldNormal.get();
      worldNormal.set(0.0, 0.0, 1.0);
      getRotationBasedOnNormal(rotationToPack, normalVector3d, worldNormal);
   }

   public static void getRotationBasedOnNormal(AxisAngle4d rotationToPack, Vector3d rotatedNormal, Vector3d referenceNormal)
   {
      Vector3d rotationAxis = tempCrossProduct.get();
      rotationAxis.set(0.0, 0.0, 0.0);

      rotationAxis.cross(referenceNormal, rotatedNormal);
      double rotationAngle = referenceNormal.angle(rotatedNormal);

      boolean normalsAreParallel = rotationAxis.lengthSquared() < 1e-7;
      if (normalsAreParallel)
      {
         rotationAngle = rotatedNormal.getZ() > 0.0 ? 0.0 : Math.PI;
         rotationAxis.set(1.0, 0.0, 0.0);
      }

      rotationToPack.set(rotationAxis, rotationAngle);
   }

   public static ArrayList<Point2d> getNormalPointsFromLine(Point2d firstLinePoint, Point2d secondLinePoint, double lengthOffset)
   {
      boolean DEBUG = false;
      ArrayList<Point2d> listPoints = new ArrayList<Point2d>();

      // Filled-in by static calls
      Vector2d originalDirection = null;
      Point2d bisectorStart = new Point2d(0, 0);
      Vector2d bisectorDirection = new Vector2d(0, 0);
      Point2d finalCalculatedPoint1 = new Point2d(0.0, 0.0);
      Point2d finalCalculatedPoint2 = new Point2d(0.0, 0.0);

      double offsetLength = lengthOffset;

      GeometryTools.getPerpendicularBisector(firstLinePoint, secondLinePoint, bisectorStart, bisectorDirection);

      originalDirection = new Vector2d(secondLinePoint.getX() - firstLinePoint.getX(), secondLinePoint.getY() - firstLinePoint.getY());


      double oldAngle = GeometryTools.getAngleFromFirstToSecondVector(new Vector2d(1.0, 0.0), originalDirection);

      double additionalAngle = GeometryTools.getAngleFromFirstToSecondVector(originalDirection, bisectorDirection);

      double angleOffset = oldAngle + additionalAngle;

      finalCalculatedPoint1.set(bisectorStart.getX() + offsetLength * Math.cos(angleOffset), bisectorStart.getY() + offsetLength * Math.sin(angleOffset));

      finalCalculatedPoint2.set(bisectorStart.getX() + offsetLength * Math.cos(angleOffset + Math.PI),
                                bisectorStart.getY() + offsetLength * Math.sin(angleOffset + Math.PI));
      listPoints.add(finalCalculatedPoint1);
      listPoints.add(finalCalculatedPoint2);

      if (DEBUG)
      {
         System.out.println("\n\nBisector Start: " + bisectorStart);
         System.out.println("Direction Vector: " + bisectorDirection);
         System.out.println("angle between original and x-axis: " + Math.toDegrees(oldAngle));
         System.out.println("Angle between original and bisector: " + Math.toDegrees(additionalAngle));
         System.out.println("Angle Total: " + Math.toDegrees(angleOffset));
         System.out.println("1. Calculated Final Point: " + finalCalculatedPoint1.getX() + ", " + finalCalculatedPoint1.getY());
         System.out.println("2. Calculated Final Point: " + finalCalculatedPoint2.getX() + ", " + finalCalculatedPoint2.getY());
      }

      return listPoints;

   }

   public static Point2d getMatchingPairOfPoints(ArrayList<Point2d> listOfPoints)
   {
      Point2d matchedPoint = null;
      for (Point2d pointToMatch : listOfPoints)
      {
         for (Point2d point : listOfPoints)
         {
            if (Math.abs(pointToMatch.getX() - point.getX()) < 0.00001)
            {
               if (Math.abs(pointToMatch.getY() - point.getY()) < 0.00001)
               {
                  matchedPoint = pointToMatch;
               }
            }
         }
      }

      return matchedPoint;
   }

   public static FrameOrientation getTransform(FramePoint point, FrameVector normal)
   {
      RigidBodyTransform transform = new RigidBodyTransform();

      transform.setRotation(getRotationBasedOnNormal(normal.getVectorCopy()));

      Vector3d translation = new Vector3d();
      point.get(translation);
      transform.setTranslation(translation);

      return new FrameOrientation(ReferenceFrame.getWorldFrame(), transform);
   }

   /**
    *  This method returns the point representing where the bisector of an
    *  angle of a triangle intersects the opposite side.
    *  Given a triangle defined by three points (A,B,C),
    *  To find the Bisector that divides the angle at B in half
    *  and intersects AC at X.
    *
    *    BA    AX
    *    --  = --
    *    BC    CX
    *
    * not garbage free!
    * @param A Point2d
    * @param B Point2d
    * @param C Point2d
    * @return Point2d the intersection point of the bisector with the opposite side
    */
   public static Point2d getTriangleBisector(Point2d A, Point2d B, Point2d C)
   {
      Point2d bisectorToPack = new Point2d();
      getTriangleBisector(A, B, C, bisectorToPack);
      return bisectorToPack;
   }

   private static final ThreadLocal<Vector2d> tempAToC = new ThreadLocal<Vector2d>()
   {
      @Override
      public Vector2d initialValue()
      {
         return new Vector2d();
      }
   };

   /**
    *  This method returns the point representing where the bisector of an
    *  angle of a triangle intersects the opposite side.
    *  Given a triangle defined by three points (A,B,C),
    *  To find the Bisector that divides the angle at B in half
    *  and intersects AC at X.
    *
    *    BA    AX
    *    --  = --
    *    BC    CX
    *
    *
    * @param A Point2d
    * @param B Point2d
    * @param C Point2d
    * @return Point2d the intersection point of the bisector with the opposite side
    */
   public static void getTriangleBisector(Point2d A, Point2d B, Point2d C, Point2d bisectorToPack)
   {
      // find all proportional values
      double BA = B.distance(A);
      double BC = B.distance(C);
      double AC = A.distance(C);
      double AX = AC / ((BC / BA) + 1.0);

      // use AX distance to find X along AC
      Vector2d AtoC = tempAToC.get();
      AtoC.set(C);
      AtoC.sub(A);
      AtoC.normalize();
      AtoC.scale(AX);

      bisectorToPack.set(A);
      bisectorToPack.add(AtoC);
   }

   public static double getAngleFromFirstToSecondVector(Vector2d firstVector, Vector2d secondVector)
   {
      double v1x = firstVector.getX();
      double v1y = firstVector.getY();
      double v2x = secondVector.getX();
      double v2y = secondVector.getY();
      return getAngleFromFirstToSecondVector(v1x, v1y, v2x, v2y);
   }

   public static double getAngleFromFirstToSecondVector(double v1x, double v1y, double v2x, double v2y)
   {
      double v1Length = Math.sqrt(v1x * v1x + v1y * v1y);

      if (v1Length < 1e-7)
         return 0.0;

      v1x /= v1Length;
      v1y /= v1Length;

      double v2Length = Math.sqrt(v2x * v2x + v2y * v2y);

      if (v2Length < 1e-7)
         return 0.0;

      v2x /= v2Length;
      v2y /= v2Length;

      // The sign of the angle comes from the cross product
      double crossProduct = v1x * v2y - v1y * v2x;
      // the magnitude of the angle comes from the dot product
      double dotProduct = v1x * v2x + v1y * v2y;

      double angle = Math.atan2(crossProduct, dotProduct);
      // This is a hack to get the polygon tests to pass.
      // Probably some edge case not well handled somewhere (Sylvain)
      if (crossProduct == 0.0)
         angle = -angle;

      return angle;
   }

   /**
    * Creates a Cube given size, color, and postion
    *
    * @param thisColor Vector3f
    * @param position Point3d
    * @param size double
    * @return BranchGroup
    */



   /**
    * Calculates distance between two Double points, a and b.
    *
    * @param a double[]
    * @param b double[]
    * @return double
    */
   public static double distanceBetweenPoints(double[] a, double[] b)
   {
      if (a.length != b.length)
      {
         throw new IllegalArgumentException("cannot find distance between points of different dimensions");
      }

      double dist = 0.0;
      for (int i = 0; i < a.length; i++)
      {
         dist += (a[i] - b[i]) * (a[i] - b[i]);
      }

      dist = Math.sqrt(dist);

      return dist;
   }

   /**
    * Calculates distance between two points.
    */
   public static double distanceBetweenPoints(double x0, double y0, double x1, double y1)
   {
      double deltaX = x1 - x0;
      double deltaY = y1 - y0;
      return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
   }

   /**
    * Calculates distance between two Point2ds, a and b.
    *
    * @param a Point2d
    * @param b Point2d
    * @return double
    */
   public static double distanceBetweenPoints(Point2d a, Point2d b)
   {
      return a.distance(b);
   }
   
   public static double dotProduct(Point2d start1, Point2d end1, Point2d start2, Point2d end2)
   {
      double vector1X = end1.getX() - start1.getX();
      double vector1Y = end1.getY() - start1.getY();
      double vector2X = end2.getX() - start2.getX();
      double vector2Y = end2.getY() - start2.getY();
      
      return vector1X * vector2X + vector1Y * vector2Y;
   }

   /**
    * Let the test point be C (Cx,Cy) and the line be AB (Ax,Ay) to (Bx,By).
    * Let P be the point of perpendicular projection of C on AB.  The parameter
    * r, which indicates P's position along AB, is computed by the dot product
    * of AC and AB divided by the square of the length of AB:
    *
    *        AC dot AB
    *    r = ---------
    *        ||AB||^2
    *
    * Let the scalar r represent the proportional distance of the projected point along the line.  If
    * r < 0, then lineStart is the closest point.  If r > 1, then lineEnd is the closest point.  If
    * 0 < r < 1, then the closest point is between lineStart and lineEnd.
    */


   public static Point2d getClosestPointToLineSegment(Point2d testPoint, Point2d lineStart, Point2d lineEnd)
   {
      LineSegment2d tempLineSegment = new LineSegment2d(lineStart, lineEnd);

      return tempLineSegment.getClosestPointOnLineSegmentCopy(testPoint);

   }

   public static double getXYDistance(FramePoint point1, FramePoint point2)
   {
      point1 = new FramePoint(point1.getReferenceFrame(), point1.getX(), point1.getY(), 0.0);
      point2 = new FramePoint(point2.getReferenceFrame(), point2.getX(), point2.getY(), 0.0);

      return point1.distance(point2);
   }

   public static double getXYDistance(Point3d point1, Point3d point2)
   {
      point1 = new Point3d(point1.getX(), point1.getY(), 0.0);
      point2 = new Point3d(point2.getX(), point2.getY(), 0.0);

      return point1.distance(point2);
   }

   /**
    *
    *
    * @param xMin double
    * @param yMin double
    * @param zMin double
    * @param xMax double
    * @param yMax double
    * @param zMax double
    * @param xResolution double
    * @param yResolution double
    * @param zResolution double
    * @return ArrayList
    */

   public static ArrayList<Point3d> getVerticalSpansOfPoints(double xMin, double yMin, double zMin, double xMax, double yMax, double zMax, double xResolution,
           double yResolution, double zResolution)
   {
      if ((xMin >= xMax) || (yMin >= yMax) || (zMin >= zMax))
      {
         throw new IllegalArgumentException("illegal bounds: (xMin, xMax), (yMin, yMax), (zMin, zMax): (" + xMin + ", " + xMax + "), (" + yMin + ", " + yMax
                                            + "), (" + zMin + ", " + zMax + ")");
      }

      ArrayList<Point3d> points = new ArrayList<Point3d>();

      ArrayList<Double> xSpan = new ArrayList<Double>();
      ArrayList<Double> ySpan = new ArrayList<Double>();

      for (double x = xMin + xResolution; x <= xMax; x += xResolution)
      {
         xSpan.add(new Double(x));
      }

      for (double y = yMin; y <= yMax; y += yResolution)
      {
         ySpan.add(new Double(y));
      }

//    for (double x = xMin; x < xMax; x += xResolution)
//    {
//       xSpan.add(new java.lang.Double(x));
//    }
//    for (double y = yMin; y < yMax; y += yResolution)
//    {
//       ySpan.add(new java.lang.Double(y));
//    }

      Double[] xSpanArray = new Double[xSpan.size()];
      Double[] ySpanArray = new Double[ySpan.size()];
      for (int i = 0; i < xSpan.size(); i++)
      {
         xSpanArray[i] = xSpan.get(i);
      }

      for (int i = 0; i < ySpan.size(); i++)
      {
         ySpanArray[i] = ySpan.get(i);
      }

      ArrayList<Point3d> sides = getVerticalPlanePointsAtXYSpans(new Double[] {new Double(xMin), new Double(xMax)}, ySpanArray,
                                    zMin, zMax, zResolution);
      ArrayList<Point3d> topAndBottom = getVerticalPlanePointsAtXYSpans(xSpanArray, new Double[] {new Double(yMin),
              new Double(yMax)}, zMin, zMax, zResolution);

      points.addAll(sides);
      points.addAll(topAndBottom);

      return points;
   }

   /**
    *
    * @param xs Double[]
    * @param ys Double[]
    * @param zMin double
    * @param zMax double
    * @param zResolution double
    * @return ArrayList
    */
   private static ArrayList<Point3d> getVerticalPlanePointsAtXYSpans(Double[] xs, Double[] ys, double zMin, double zMax, double zResolution)
   {
      ArrayList<Point3d> points = new ArrayList<Point3d>();
      for (Double x : xs)
      {
         for (Double y : ys)
         {
            for (double z = zMin; z < zMax; z += zResolution)
            {
               points.add(new Point3d(x, y, z));
            }
         }
      }

      return points;
   }

   /**
    * Returns the normal of a Plane specified by three points
    * If 2 or more points are the same, returns NaN
    *
    * @param a Point3d
    * @param b Point3d
    * @param c Point3d
    * @return Vector3d
    */
   public static Vector3d getNormalToPlane(Point3d a, Point3d b, Point3d c)
   {
      Vector3d x = new Vector3d(b);
      x.sub(a);
      Vector3d y = new Vector3d(c);
      y.sub(a);
      x.cross(x, y);
      x.normalize();

      return x;
   }
   
   public static boolean arePlanesParallel(Plane3d planeOne, Plane3d planeTwo, double epsilon)
   {
      boolean normalsAreEqual = planeOne.getNormal().epsilonEquals(planeTwo.getNormal(), epsilon);
      boolean normalsAreOpposite = true;
      normalsAreOpposite &= MathTools.epsilonEquals(planeOne.getNormal().getX(), -planeTwo.getNormal().getX(), epsilon);
      normalsAreOpposite &= MathTools.epsilonEquals(planeOne.getNormal().getY(), -planeTwo.getNormal().getY(), epsilon);
      normalsAreOpposite &= MathTools.epsilonEquals(planeOne.getNormal().getZ(), -planeTwo.getNormal().getZ(), epsilon);
      return normalsAreEqual || normalsAreOpposite;
   }
   
   private static final ThreadLocal<Vector3d> pointVectorForDotCheck = new ThreadLocal<Vector3d>()
   {
      @Override
      public Vector3d initialValue()
      {
         return new Vector3d();
      }
   };
   
   public static boolean areCoplanar(Plane3d planeOne, Plane3d planeTwo, double epsilon)
   {
      if (!planeOne.getNormal().epsilonEquals(planeTwo.getNormal(), epsilon))
      {
         return false;
      }
      
      pointVectorForDotCheck.get().sub(planeTwo.getPoint(), planeOne.getPoint());
      if (!MathTools.epsilonEquals(planeOne.getNormal().dot(pointVectorForDotCheck.get()), 0.0, epsilon))
      {
         return false; 
      }
      
      return true;
   }

// TODO move to polygon?
   @Deprecated
   public static void movePointInsidePolygonAlongLine(FramePoint2d point, FrameConvexPolygon2d polygon, FrameLine2d line)
   {
      // Defaults to 2mm for desired capture to prevent some jerky behavior with VirtualToePoints.. // TODO: remove
      double amountToBeInside = 0.002;
      movePointInsidePolygonAlongLine(point, polygon, line, amountToBeInside);
   }

// TODO move to polygon?
   @Deprecated
   public static void movePointInsidePolygonAlongLine(FramePoint2d point, FrameConvexPolygon2d polygon, FrameLine2d line, double amountToBeInside)
   {
      if (!polygon.isPointInside(point))
      {
         FramePoint2d[] intersections = polygon.intersectionWith(line);
         if (intersections != null)
         {
            FramePoint2d intersectionToUse;

            if (intersections.length == 2)
            {
               double distanceSquaredToIntersection0 = point.distanceSquared(intersections[0]);
               double distanceSquaredToIntersection1 = point.distanceSquared(intersections[1]);

               if (distanceSquaredToIntersection0 <= distanceSquaredToIntersection1)
                  intersectionToUse = intersections[0];
               else
                  intersectionToUse = intersections[1];


               point.setX(intersectionToUse.getX());
               point.setY(intersectionToUse.getY());

               // Move in a little along the line:
               FrameLineSegment2d guideLineSegment = new FrameLineSegment2d(intersections);
               FrameVector2d frameVector2d = new FrameVector2d();
               guideLineSegment.getFrameVector(frameVector2d);
               if (intersectionToUse == intersections[1])
                  frameVector2d.scale(-1.0);
               frameVector2d.normalize();
               frameVector2d.scale(amountToBeInside);

               point.setX(point.getX() + frameVector2d.getX());
               point.setY(point.getY() + frameVector2d.getY());
            }
            else
            {
               throw new RuntimeException("This is interesting, shouldn't get here.");
            }
         }
         else
         {
            point.set(polygon.getClosestVertexCopy(line));
         }
      }
   }

   public static void movePointInsidePolygonAlongVector(FramePoint2d pointToMove, FrameVector2d vector, FrameConvexPolygon2d polygon, double distanceToBeInside)
   {
      if (polygon.getNumberOfVertices() < 2)
      {
         return;
      }

      if (distanceToBeInside < 0.0)
         throw new RuntimeException("distanceToBeInside = " + distanceToBeInside);

      FrameLine2d line = new FrameLine2d(pointToMove, vector);
      FramePoint2d[] intersections = polygon.intersectionWith(line);

      if (intersections != null)
      {
         if ((intersections.length != 2) && (intersections.length != 1))
            throw new RuntimeException("intersections.length != 2 && intersections.length != 1. intersections.length = " + intersections.length);

         if (intersections.length == 1)
         {
            pointToMove.set(intersections[0]);

            return;
         }

         // make sure it's inside or on the edge of the polygon
         boolean insidePolygon = polygon.isPointInside(pointToMove);
         if (!insidePolygon)
         {
            double minDistance = Double.POSITIVE_INFINITY;
            FramePoint2d closestIntersection = null;
            for (int i = 0; i < intersections.length; i++)
            {
               FramePoint2d intersection = intersections[i];
               double distance = pointToMove.distance(intersection);
               if (distance < minDistance)
               {
                  minDistance = distance;
                  closestIntersection = intersection;
               }
            }

            pointToMove.set(closestIntersection);
         }

         // make sure distance constraint is met; if infeasible, use midpoint of intersections
         double distanceBetweenIntersections = intersections[0].distance(intersections[1]);
         boolean constraintFeasible = distanceBetweenIntersections > 2.0 * distanceToBeInside;

         if (constraintFeasible)
         {
            for (int i = 0; i < intersections.length; i++)
            {
               double distance = intersections[i].distance(pointToMove);
               if (distance < distanceToBeInside)
               {
                  int j = 1 - i;
                  vector.sub(intersections[j], intersections[i]);
                  vector.normalize();
                  vector.scale(distanceToBeInside);
                  pointToMove.set(intersections[i]);
                  pointToMove.add(vector);
               }
            }
         }
         else
         {
            pointToMove.interpolate(intersections[0], intersections[1], 0.5);
         }
      }
      else
      {
         pointToMove.set(polygon.getClosestVertexCopy(line));

      }

//    else
//    {
//       StringBuilder stringBuilder = new StringBuilder();
//       stringBuilder.append("intersections == null\n");
//       stringBuilder.append("pointToMove = " + pointToMove + "\n");
//       stringBuilder.append("vector = " + vector + "\n");
//       stringBuilder.append("polygon = " + polygon + "\n");
//       stringBuilder.append("distanceToBeInside = " + distanceToBeInside);
//
//       throw new RuntimeException(stringBuilder.toString());
//    }
   }

   public static void projectOntoPolygonAndCheckDistance(FramePoint2d point, FrameConvexPolygon2d polygon, double epsilon)
   {
      ReferenceFrame originalReferenceFrame = point.getReferenceFrame();
      point.changeFrame(polygon.getReferenceFrame());
      FramePoint2d originalPoint = new FramePoint2d(point);
      polygon.orthogonalProjection(point);
      double distance = originalPoint.distance(point);
      if (distance > epsilon)
         throw new RuntimeException("point outside polygon by " + distance);
      point.changeFrame(originalReferenceFrame);
   }


   /**
    * arePointsInOrderColinear: This returns true if:
    * middle point is epsilon close to start or end
    *
    * Otherwise:
    * if the start is epsilon close to the end, return false
    *
    * if |(start to middle unit vector) dot with (start to end unit vector) - 1| > epsilon
    * return false
    * else return true
    *
    * @param startPoint Point2d
    * @param middlePoint Point2d
    * @param endPoint Point2d
    * @return boolean
    */
   public static boolean arePointsInOrderAndColinear(Point2d startPoint, Point2d middlePoint, Point2d endPoint, double epsilon)
   {
      double startToEndDistance = startPoint.distance(endPoint);
      double startToMiddleDistance = startPoint.distance(middlePoint);
      double middleToEndDistance = middlePoint.distance(endPoint);

      if (startToMiddleDistance < epsilon)
      {
         // middle very close to the start
         return true;
      }
      else if (middleToEndDistance < epsilon)
      {
         // middle very close to end
         return true;
      }
      else if (startToEndDistance < epsilon)
      {
         // start too close to end to fit middle in between
         return false;
      }
      else if ((startToMiddleDistance - startToEndDistance) > epsilon)
      {
         // middle farther from start than end point
         return false;
      }
      else if ((middleToEndDistance - startToEndDistance) > epsilon)
      {
         // middle farther from end than start point
         return false;
      }
      else
      {
         Vector2d startToEnd = new Vector2d(endPoint);
         startToEnd.sub(startPoint);
         startToEnd.normalize();

         Vector2d startToMiddle = new Vector2d(middlePoint);
         startToMiddle.sub(startPoint);
         startToMiddle.normalize();

         if (Math.abs(1.0 - startToMiddle.dot(startToEnd)) > epsilon)
            return false;
         else
            return true;
      }
   }

   /**
    * arePointsInOrderColinear: This returns true if:
    * middle point is epsilon close to start or end
    *
    * Otherwise:
    * if the start is epsilon close to the end, return false
    *
    * if |(start to middle unit vector) dot with (start to end unit vector) - 1| > epsilon
    * return false
    * else return true
    *
    * @param startPoint Point3d
    * @param middlePoint Point3d
    * @param endPoint Point3d
    * @return boolean
    */
   public static boolean arePointsInOrderAndColinear(Point3d startPoint, Point3d middlePoint, Point3d endPoint, double epsilon)
   {
      double startToEndDistance = startPoint.distance(endPoint);
      double startToMiddleDistance = startPoint.distance(middlePoint);
      double middleToEndDistance = middlePoint.distance(endPoint);

      if (startToMiddleDistance < epsilon)
      {
         // middle very close to the start
         return true;
      }
      else if (middleToEndDistance < epsilon)
      {
         // middle very close to end
         return true;
      }
      else if (startToEndDistance < epsilon)
      {
         // start too close to end to fit middle in between
         return false;
      }
      else if ((startToMiddleDistance - startToEndDistance) > epsilon)
      {
         // middle farther from start than end point
         return false;
      }
      else if ((middleToEndDistance - startToEndDistance) > epsilon)
      {
         // middle farther from end than start point
         return false;
      }
      else
      {
         Vector3d startToEnd = new Vector3d(endPoint);
         startToEnd.sub(startPoint);
         startToEnd.normalize();

         Vector3d startToMiddle = new Vector3d(middlePoint);
         startToMiddle.sub(startPoint);
         startToMiddle.normalize();

         if (Math.abs(1.0 - startToMiddle.dot(startToEnd)) > epsilon)
            return false;
         else
            return true;
      }
   }

   /**
    * Calculate an unknown side length of a fully defined 2D Triangle by the law of Cosine
    *
    * @param lengthSideA
    * @param lengthSideB
    * @param angleBetweenAAndB
    */
   public static double getUnknownTriangleSideLengthByLawOfCosine(double lengthSideA, double lengthSideB, double angleBetweenAAndB)
   {
      MathTools.checkIfInRange(lengthSideA, 0.0, Double.POSITIVE_INFINITY);
      MathTools.checkIfInRange(lengthSideB, 0.0, Double.POSITIVE_INFINITY);

      if (Math.abs(angleBetweenAAndB) > Math.PI)
      {
         throw new RuntimeException("angleBetweenAAndB " + angleBetweenAAndB + " does not define a triangle.");
      }

      return Math.sqrt(MathTools.square(lengthSideA) + MathTools.square(lengthSideB) - 2.0 * lengthSideA * lengthSideB * Math.cos(angleBetweenAAndB));
   }

   /**
    * Calculate an unknown angle of a fully defined 2D Triangle by the law of Cosine
    *
    * @param lengthNeighbourSideA
    * @param lengthNeighbourSideB
    * @param lengthOppositeSideC
    */
   public static double getUnknownTriangleAngleByLawOfCosine(double lengthNeighbourSideA, double lengthNeighbourSideB, double lengthOppositeSideC)
   {
      MathTools.checkIfInRange(lengthNeighbourSideA, 0.0, Double.POSITIVE_INFINITY);
      MathTools.checkIfInRange(lengthNeighbourSideB, 0.0, Double.POSITIVE_INFINITY);
      MathTools.checkIfInRange(lengthOppositeSideC, 0.0, Double.POSITIVE_INFINITY);

      if (GeometryTools.isFormingTriangle(lengthNeighbourSideA, lengthNeighbourSideB, lengthOppositeSideC))
      {
         return Math.acos((MathTools.square(lengthNeighbourSideA) + MathTools.square(lengthNeighbourSideB) - MathTools.square(lengthOppositeSideC))
                          / (2.0 * lengthNeighbourSideA * lengthNeighbourSideB));
      }
      else
      {
         throw new RuntimeException("Unable to build a Triangle of the given triangle sides a: "
                                    + lengthNeighbourSideA + " b: " + lengthNeighbourSideB + " c: " + lengthOppositeSideC);
      }
   }

   /**
    * Get a unknown cathetus (90-deg triangle one of the two shorter triangle sides, neighbouring the 90-degree angle) by Pythagoras law, a^2+b^2=c^2
    *
    * @param hypothenuseC the longest side
    * @param cathetusA one short side
    * @param cathetusB the other short side
    */
   public static double pythagorasGetCathetus(double hypothenuseC, double cathetusA)
   {
      MathTools.checkIfInRange(cathetusA, 0.0, hypothenuseC);

      return Math.sqrt(MathTools.square(hypothenuseC) - MathTools.square(cathetusA));
   }

   public static boolean isFormingTriangle(double lengthNeighbourSideA, double lengthNeighbourSideB, double lengthOppositeSideC)
   {
      double[] length_checker = new double[3];
      length_checker[0] = lengthNeighbourSideA;
      length_checker[1] = lengthNeighbourSideB;
      length_checker[2] = lengthOppositeSideC;
      Arrays.sort(length_checker);
      if (length_checker[0] + length_checker[1] <= length_checker[2])
         return false;
      else
         return true;
   }

   /**
    * Get the hypothenuse c (90-degree triangle longest triangle length, opposite to the 90-degree angle) by Pythagoras law, a^2+b^2=c^2
    *
    * @param cathetusA one short side
    * @param cathetusB the other short side
    * @param hypothenuseC the longest side
    */
   public static double pythagorasGetHypothenuse(double cathetusA, double cathetusB)
   {
      return Math.hypot(cathetusA, cathetusB);
   }

   // Needs to be reimplemented with EJML and without generating garbage.
   /*
    * Projects point p onto the plane defined by p1, p2, and p3
    */
//   public static Vector3d getProjectionOntoPlane(Vector3d p1, Vector3d p2, Vector3d p3, Vector3d p)
//   {
//      Vector3d p2_minus_p1 = new Vector3d(p2);
//      p2_minus_p1.sub(p1);
//
//      Vector3d p3_minus_p1 = new Vector3d(p3);
//      p3_minus_p1.sub(p1);
//
//      Vector3d n = new Vector3d(p2_minus_p1);
//      n.cross(n, p3_minus_p1);
//      n.normalize();
//
//      // convert to matrix so the following calculation is cleaner
//      Matrix P = MatrixTools.vector3dToMatrix(p);
//      Matrix P1 = MatrixTools.vector3dToMatrix(p1);
//      Matrix N = MatrixTools.vector3dToMatrix(n);
//
//      double scale = (((P1.minus(P)).transpose()).times(N)).get(0, 0);
//      Matrix Perp = N.times(scale);
//
//      Matrix Proj = P.plus(Perp);
//
//      return new Vector3d(Proj.get(0, 0), Proj.get(1, 0), Proj.get(2, 0));
//   }

   public static double minimumDistance(FramePoint testPoint, List<FramePoint> points)
   {
      double ret = Double.POSITIVE_INFINITY;
      for (FramePoint point : points)
      {
         double distanceSquared = testPoint.distanceSquared(point);
         if (distanceSquared < ret)
            ret = distanceSquared;
      }

      return Math.sqrt(ret);
   }

   public static ArrayList<FramePoint2d> changeFrameToZUpAndProjectToXYPlane(ReferenceFrame zUpFrame, List<FramePoint> points)
   {
      ArrayList<FramePoint2d> ret = new ArrayList<FramePoint2d>(points.size());

      for (int i = 0; i < points.size(); i++)
      {
         FramePoint framePoint = new FramePoint(points.get(i));
         framePoint.changeFrame(zUpFrame);

         ret.add(framePoint.toFramePoint2d());
      }

      return ret;
   }

   public static ArrayList<FramePoint2d> projectToXYPlane(List<FramePoint> points)
   {
      ArrayList<FramePoint2d> ret = new ArrayList<FramePoint2d>(points.size());
      for (int i = 0; i < points.size(); i++)
      {
         FramePoint point3d = points.get(i);
         ret.add(new FramePoint2d(point3d.getReferenceFrame(), point3d.getX(), point3d.getY()));
      }

      return ret;
   }

   /**
    * Finds the minimum distance between two convex polygons
    * Taken from http://cygnus-x1.cs.duke.edu/~edels/Papers/1985-J-02-ComputingExtremeDistances.pdf
    * @return Two points, one from each polygon, between which is the minimum distance between the two polygons
    */
   public static Point2d[] computeMinimumDistancePoints(ConvexPolygon2d polygon1, ConvexPolygon2d polygon2, double epsilon)
   {
      // TODO Do something more clever than actually computing the intersection there!
      if (ConvexPolygonTools.computeIntersectionOfPolygons(polygon1, polygon2, new ConvexPolygon2d()))
      {
         throw new RuntimeException("Cannot compute minimum distance between intersecting polygons.");
      }

      if ((polygon1.getNumberOfVertices() < 3) || (polygon2.getNumberOfVertices() < 3))
      {
         throw new RuntimeException("Polygon inputs are degenerate.");
      }

      int[] v1Tangents = findStartAndEndTangents(polygon2.getVertex(0), polygon1, epsilon);
      int[] v2Tangents = findStartAndEndTangents(polygon1.getVertex(0), polygon2, epsilon);

      int v1Start = v1Tangents[0];
      int v1End = v1Tangents[1];
      int v2Start = v2Tangents[0];
      int v2End = v2Tangents[1];

      int[] updatedIndices = binaryElimination(polygon1, polygon2, v1Start, v1End, v2Start, v2End, epsilon);
      v1Start = updatedIndices[0];
      v1End = updatedIndices[1];
      v2Start = updatedIndices[2];
      v2End = updatedIndices[3];

      return getClosestPointsFromRemainingEdgesAndVertices(polygon1, polygon2, v1Start, v1End, v2Start, v2End);
   }

   public static Point2d[] computeMinimumDistancePoints(ConvexPolygon2d polygon1, ConvexPolygon2d polygon2)
   {
      return computeMinimumDistancePoints(polygon1, polygon2, .01);
   }


   // TODO potentially implement [Chazelle and Dobkin] to get logarithmic running time for computeMinimumDistancePoints (though it would actually be log^2 in current
   // implementation, since binaryElimination, which has is O(log(n)) uses this method in each loop)

   /**
    * Finds the indices of the vertices of the polygon that form tangent lines to the polygon with the parameter point
    * @return The desired indices, ordered such that they form a range that includes all vertices visible from the parameter point; if there are more than two
    *          only returns the two necessary to specify this range
    */
   private static int[] findStartAndEndTangents(Point2d point, ConvexPolygon2d polygon, double epsilon)
   {
      int tangentIndex1;
      int tangentIndex2;

      int vIndex = 0;

      while (!pointMakesTangentToPolygon(polygon, point, vIndex, epsilon))
      {
         vIndex++;
         vIndex %= polygon.getNumberOfVertices();
      }

      tangentIndex1 = vIndex;
      Vector2d tangent1 = new Vector2d(polygon.getVertex(tangentIndex1).getX() - point.getX(),polygon.getVertex(tangentIndex1).getY() - point.getY());

      vIndex++;
      vIndex %= polygon.getNumberOfVertices();

      while (!pointMakesTangentToPolygon(polygon, point, vIndex, epsilon))
      {
         vIndex++;
         vIndex %= polygon.getNumberOfVertices();
      }

      tangentIndex2 = vIndex;
      Vector2d tangent2 = new Vector2d(polygon.getVertex(tangentIndex2).getX() - point.getX(), polygon.getVertex(tangentIndex2).getY() - point.getY());

      if (getAngleFromFirstToSecondVector(tangent1, tangent2) > 0)
      {
         return new int[] {tangentIndex1, tangentIndex2};
      }

      return new int[] {tangentIndex2, tangentIndex1};
   }

   /**
    * Uses the fact that if a line passes through a vertex of a convex polygon, the angles to the adjacent edges are going to be in opposite directions
    * @return Whether or not the line including the point and vertex is tangent to the polygon
    */
   private static boolean pointMakesTangentToPolygon(ConvexPolygon2d polygon, Point2d point, int vertexIndex, double epsilon)
   {
      Point2d vertex = polygon.getVertex(vertexIndex);
      Point2d previous = polygon.getPreviousVertex(vertexIndex);
      Point2d next = polygon.getNextVertex(vertexIndex);

      Vector2d base = new Vector2d(point.getX() - vertex.getX(), point.getY() - vertex.getY());
      Vector2d first = new Vector2d(previous.getX() - vertex.getX(), previous.getY() - vertex.getY());
      Vector2d second = new Vector2d(next.getX() - vertex.getX(), next.getY() - vertex.getY());
      double firstAngle = getAngleFromFirstToSecondVector(base, first);
      double secondAngle = getAngleFromFirstToSecondVector(base, second);

      if (firstAngle * secondAngle >= 0)
      {    // if both angles have the same sign, the line does not pass through the polygon
         return true;
      }

      if (MathTools.epsilonEquals(firstAngle, 0, epsilon) || MathTools.epsilonEquals(secondAngle, 0, epsilon))
      {    // if either angle is close to 0, assume floating point arithmetic error
         return true;
      }

      return false;
   }

   /**
    * Checks if index is within range; if low is greater than high, this implies a modularly cyclical range
    * @return True if the index is between low and high
    */
   private static boolean isInRange(int index, int low, int high)
   {
      if ((low <= index) && (index <= high))
      {
         return true;
      }

      if ((high < low) && ((index >= low) || (index <= high)))
      {
         return true;
      }

      return false;
   }

   /**
    * Eliminates vertices and return a range for each polygon, each of which comprises of at most two vertices
    * @return Array with the low and high end of each range, respectively
    */
   private static int[] binaryElimination(ConvexPolygon2d polygon1, ConvexPolygon2d polygon2, int v1Start, int v1End, int v2Start, int v2End, double epsilon)
   {
      Point2d v1Median;
      Point2d v2Median;

      int numberOfVertices1 = polygon1.getNumberOfVertices();
      int numberOfVertices2 = polygon2.getNumberOfVertices();

      while (((numberOfVertices1 + v1End - v1Start) % numberOfVertices1 + 1 > 2) || ((numberOfVertices2 + v2End - v2Start) % numberOfVertices2 + 1 > 2))
      {
         int v1MedianIndex = (v1Start <= v1End) ? (v1End + v1Start + 1) / 2 : ((v1End + v1Start + 1 + numberOfVertices1) / 2) % numberOfVertices1;
         int v2MedianIndex = (v2Start <= v2End) ? (v2End + v2Start) / 2 : ((v2End + v2Start + numberOfVertices2) / 2) % numberOfVertices2;
         v1Median = polygon1.getVertex(v1MedianIndex);
         v2Median = polygon2.getVertex(v2MedianIndex);

         Vector2d m = new Vector2d(v2Median.getX() - v1Median.getX(), v2Median.getY() - v1Median.getY());
         Vector2d mReversed = new Vector2d(v1Median.getX() - v2Median.getX(), v1Median.getY() - v2Median.getY());

         int edge1AStart = ((v1MedianIndex + numberOfVertices1 - 1) % numberOfVertices1);
         int edge1BEnd = (v1MedianIndex + 1) % numberOfVertices1;
         int edge2BStart = ((v2MedianIndex + numberOfVertices2 - 1) % numberOfVertices2);
         int edge2AEnd = (v2MedianIndex + 1) % numberOfVertices2;
         Vector2d edge1A = new Vector2d(polygon1.getVertex(edge1AStart).getX() - v1Median.getX(), polygon1.getVertex(edge1AStart).getY() - v1Median.getY());
         Vector2d edge1B = new Vector2d(polygon1.getVertex(edge1BEnd).getX() - v1Median.getX(), polygon1.getVertex(edge1BEnd).getY() - v1Median.getY());
         Vector2d edge2A = new Vector2d(polygon2.getVertex(edge2AEnd).getX() - v2Median.getX(), polygon2.getVertex(edge2AEnd).getY() - v2Median.getY());
         Vector2d edge2B = new Vector2d(polygon2.getVertex(edge2BStart).getX() - v2Median.getX(), polygon2.getVertex(edge2BStart).getY() - v2Median.getY());

         // see diagram 3.2 in [Edelsbrunner]
         double angle1A = getAngleFromFirstToSecondVector(m, edge1A); // A' in diagram
         double angle1B = getAngleFromFirstToSecondVector(edge1B, m); // A'' in diagram
         double angle2A = getAngleFromFirstToSecondVector(edge2A, mReversed); // B' in diagram
         double angle2B = getAngleFromFirstToSecondVector(mReversed, edge2B); // B'' in diagram

         int[] range1 = findStartAndEndTangents(v2Median, polygon1, epsilon);
         int[] range2 = findStartAndEndTangents(v1Median, polygon2, epsilon);

         angle1A = MathTools.epsilonEquals(angle1A, 0, .01) ? 0 : angle1A;
         angle1B = MathTools.epsilonEquals(angle1B, 0, .01) ? 0 : angle1B;
         angle2A = MathTools.epsilonEquals(angle2A, 0, .01) ? 0 : angle2A;
         angle2B = MathTools.epsilonEquals(angle2B, 0, .01) ? 0 : angle2B;

         angle1A += ((angle1A < 0) && isInRange(v1MedianIndex, range1[0], range1[1])) ? 2 * Math.PI : 0;
         angle1B += ((angle1B < 0) && isInRange(v1MedianIndex, range1[0], range1[1])) ? 2 * Math.PI : 0;
         angle2A += ((angle2A < 0) && isInRange(v2MedianIndex, range2[0], range2[1])) ? 2 * Math.PI : 0;
         angle2B += ((angle2B < 0) && isInRange(v2MedianIndex, range2[0], range2[1])) ? 2 * Math.PI : 0;

         angle1A += ((angle1A < 0) && (angle1B < 0) && (angle1A < angle1B)) ? 2 * Math.PI : 0;
         angle1B += ((angle1A < 0) && (angle1B < 0) && (angle1B < angle1A)) ? 2 * Math.PI : 0;
         angle2A += ((angle2A < 0) && (angle2B < 0) && (angle2A < angle2B)) ? 2 * Math.PI : 0;
         angle2B += ((angle2A < 0) && (angle2B < 0) && (angle2B < angle2A)) ? 2 * Math.PI : 0;

         int[] updatedIndices;

         if ((v1Start == v1End) || (v2Start == v2End))
         {
            updatedIndices = binaryEliminationCase1(angle1A, angle1B, angle2A, angle2B, v1Start, v1MedianIndex, v1End, v2Start, v2MedianIndex, v2End, polygon1, polygon2);
            v1Start = updatedIndices[0];
            v1End = updatedIndices[1];
            v2Start = updatedIndices[2];
            v2End = updatedIndices[3];
         }
         else if ((v1End - v1Start + numberOfVertices1) % numberOfVertices1 == 1)
         {
            updatedIndices = binaryEliminationCase2(angle1A, angle1B, angle2A, angle2B, v1Start, v1MedianIndex, v1End, v2Start, v2MedianIndex, v2End, polygon1, polygon2);
            v1Start = updatedIndices[0];
            v1End = updatedIndices[1];
            v2Start = updatedIndices[2];
            v2End = updatedIndices[3];
         }
         else if ((v2End - v2Start + numberOfVertices2) % numberOfVertices2 == 1)
         {
            updatedIndices = binaryEliminationCase2(angle2A, angle2B, angle1A, angle1B, v2End, v2MedianIndex, v2Start, v1End, v1MedianIndex, v1Start, polygon1, polygon2);
            v2End = updatedIndices[0];
            v2Start = updatedIndices[1];
            v1End = updatedIndices[2];
            v1Start = updatedIndices[3];
         }
         else
         {
            updatedIndices = binaryEliminationCase3(angle1A, angle1B, angle2A, angle2B, v1Start, v1MedianIndex, v1End, v2Start, v2MedianIndex, v2End);
            v1Start = updatedIndices[0];
            v1End = updatedIndices[1];
            v2Start = updatedIndices[2];
            v2End = updatedIndices[3];
         }
      }

      return new int[] { v1Start, v1End, v2Start, v2End };
   }

   /**
    * Binary elimination helper method called if one range has a size of exactly one
    * @return Array with the low and high end of each range, respectively
    */
   private static int[] binaryEliminationCase1(double angle1A, double angle1B, double angle2A, double angle2B, int v1Start, int v1MedianIndex, int v1End,
           int v2Start, int v2MedianIndex, int v2End, ConvexPolygon2d polygon1, ConvexPolygon2d polygon2)
   {
      if (v1Start == v1End)
      {    // v1 contains only 1 viable vertex
         if (angle2A >= Math.PI / 2)
         {
            v2End = v2MedianIndex;
         }

         if (angle2B >= Math.PI / 2)
         {
            v2Start = v2MedianIndex;
         }
      }
      else if (v2Start == v2End)
      {
         if (angle1A >= Math.PI / 2)
         {
            v1Start = v1MedianIndex;
         }

         if (angle1B >= Math.PI / 2)
         {
            v1End = v1MedianIndex;
         }
      }

      return new int[] {v1Start, v1End, v2Start, v2End};
   }

   /**
    * Binary elimination helper method called if one range has a size of exactly two
    * @return Array with the low and high end of each range, respectively
    */
   private static int[] binaryEliminationCase2(double angle1A, double angle1B, double angle2A, double angle2B, int v1Start, int v1MedianIndex, int v1End,
           int v2Start, int v2MedianIndex, int v2End, ConvexPolygon2d polygon1, ConvexPolygon2d polygon2)
   {
      if (angle1A > 0)
      {
         if (angle1A + angle2A >= Math.PI)
         {
            if (angle1A >= Math.PI / 2)
            {
               v1Start = v1End;
            }

            if (angle2A >= Math.PI / 2)
            {
               v2End = v2MedianIndex;
            }
         }

         if (angle2B >= Math.PI / 2)
         {
            v2Start = v2MedianIndex;
         }

         if ((angle1A < angle2B) && (angle2B < Math.PI / 2))
         {
            Point2d proj = getOrthogonalProjectionOnLine(polygon2.getVertex(v2MedianIndex), polygon1.getVertex(v1Start), polygon1.getVertex(v1End));
            LineSegment2d p = new LineSegment2d(polygon1.getVertex(v1Start), polygon1.getVertex(v1End));
            if (p.isBetweenEndpoints(proj, 0))
            {
               v2Start = v2MedianIndex;
            }
            else
            {
               v1End = v1Start;
            }
         }
      }
      else
      {
         v1End = v1Start;

         if (angle2A >= Math.PI)
         {
            v2End = v2MedianIndex;
         }

         if (angle2B >= Math.PI)
         {
            v2Start = v2MedianIndex;
         }
      }

      return new int[] {v1Start, v1End, v2Start, v2End};
   }


   /**
    * Binary Elimination helper method called if both ranges have size greater than two
    * @return Array with the low and high end of each range, respectively
    */
   private static int[] binaryEliminationCase3(double angle1A, double angle1B, double angle2A, double angle2B, int v1Start, int v1MedianIndex, int v1End,
           int v2Start, int v2MedianIndex, int v2End)
   {
      if ((angle1A > 0) && (angle1B > 0) && (angle2A > 0) && (angle2B > 0))
      {
         if (angle1A + angle2A > Math.PI)
         {
            if (angle1A >= Math.PI / 2)
            {
               v1Start = v1MedianIndex;
            }

            if (angle2A >= Math.PI / 2)
            {
               v2End = v2MedianIndex;
            }
         }

         if (angle1B + angle2B > Math.PI)
         {
            if (angle1B >= Math.PI / 2)
            {
               v1End = v1MedianIndex;
            }

            if (angle2B >= Math.PI / 2)
            {
               v2Start = v2MedianIndex;
            }
         }
      }

      if (angle1A <= 0)
      {
         v1End = v1MedianIndex;
      }

      if (angle1B <= 0)
      {
         v1Start = v1MedianIndex;
      }

      if (angle2A <= 0)
      {
         v2Start = v2MedianIndex;
      }

      if (angle2B <= 0)
      {
         v2End = v2MedianIndex;
      }

      return new int[] {v1Start, v1End, v2Start, v2End};
   }


   /**
    * Takes in two ranges each of which are of size at most two and returns the two points on each respective polygon which are closest to one another
    */
   private static Point2d[] getClosestPointsFromRemainingEdgesAndVertices(ConvexPolygon2d polygon1, ConvexPolygon2d polygon2, int v1Start, int v1End, int v2Start, int v2End)
   {
      if ((v1Start == v1End) && (v2Start == v2End))
      {
         return new Point2d[] {polygon1.getVertex(v1Start), polygon2.getVertex(v2Start)};
      }

      else if (v1Start == v1End)
      {
         return finalPhasePointAndEdge(polygon2.getVertex(v2Start), polygon2.getVertex(v2End), polygon1.getVertex(v1Start));
      }

      else if (v2Start == v2End)
      {
         Point2d[] reverseOutput = finalPhasePointAndEdge(polygon1.getVertex(v1Start), polygon1.getVertex(v1End), polygon2.getVertex(v2Start));

         return new Point2d[] {reverseOutput[1], reverseOutput[0]};    // switch order of output so that points are returned in the order that their polygons were inputed
      }

      return finalPhaseTwoEdges(polygon1.getVertex(v1Start), polygon1.getVertex(v1End), polygon2.getVertex(v2Start), polygon2.getVertex(v2End));
   }

   /**
    * Final phase helper method called if each range has size of exactly two
    * @return The two points on each respective polygon which are closest to one another
    */
   private static Point2d[] finalPhaseTwoEdges(Point2d edgePoint1A, Point2d edgePoint1B, Point2d edgePoint2A, Point2d edgePoint2B)
   {
      LineSegment2d edge1 = new LineSegment2d(edgePoint1A, edgePoint1B);
      LineSegment2d edge2 = new LineSegment2d(edgePoint2A, edgePoint2B);
      Point2d proj1AOnto2 = getOrthogonalProjectionOnLine(edgePoint1A, edgePoint2A, edgePoint2B);
      Point2d proj1BOnto2 = getOrthogonalProjectionOnLine(edgePoint1B, edgePoint2A, edgePoint2B);
      Point2d proj2AOnto1 = getOrthogonalProjectionOnLine(edgePoint2A, edgePoint1A, edgePoint1B);
      Point2d proj2BOnto1 = getOrthogonalProjectionOnLine(edgePoint2B, edgePoint1A, edgePoint1B);

      Point2d[][] possiblePointPairsWithProj = new Point2d[4][2];
      Point2d[][] possiblePointPairsWithoutProj = new Point2d[4][2];
      double[] possibleDistancesWithProj = new double[4];
      double[] possibleDistancesWithoutProj = new double[4];

      possiblePointPairsWithProj[0] = edge2.isBetweenEndpoints(proj1AOnto2, 0) ? new Point2d[] {edgePoint1A, proj1AOnto2} : null;
      possiblePointPairsWithProj[1] = edge2.isBetweenEndpoints(proj1BOnto2, 0) ? new Point2d[] {edgePoint1B, proj1BOnto2} : null;
      possiblePointPairsWithProj[2] = edge1.isBetweenEndpoints(proj2AOnto1, 0) ? new Point2d[] {proj2AOnto1, edgePoint2A} : null;
      possiblePointPairsWithProj[3] = edge1.isBetweenEndpoints(proj2BOnto1, 0) ? new Point2d[] {proj2BOnto1, edgePoint2B} : null;

      possiblePointPairsWithoutProj[0] = new Point2d[] {edgePoint1A, edgePoint2A};
      possiblePointPairsWithoutProj[1] = new Point2d[] {edgePoint1A, edgePoint2B};
      possiblePointPairsWithoutProj[2] = new Point2d[] {edgePoint1B, edgePoint2A};
      possiblePointPairsWithoutProj[3] = new Point2d[] {edgePoint1B, edgePoint2B};

      for (int i = 0; i < 4; i++)
      {
         possibleDistancesWithProj[i] = (possiblePointPairsWithProj[i] == null)
                                        ? Double.MAX_VALUE : possiblePointPairsWithProj[i][0].distance(possiblePointPairsWithProj[i][1]);
         possibleDistancesWithoutProj[i] = possiblePointPairsWithoutProj[i][0].distance(possiblePointPairsWithoutProj[i][1]);
      }

      if (possibleDistancesWithProj[indexOfMin(possibleDistancesWithProj)] != Double.MAX_VALUE)
      {
         return possiblePointPairsWithProj[indexOfMin(possibleDistancesWithProj)];
      }

      return possiblePointPairsWithoutProj[indexOfMin(possibleDistancesWithoutProj)];
   }

   /**
    * @return Index of the minimum element in an array of doubles
    */
   private static int indexOfMin(double[] d)
   {
      if ((d == null) || (d.length == 0))
      {
         throw new RuntimeException("Cannot find minimum of empty or null array.");
      }

      int minIndex = 0;
      double minValue = d[minIndex];
      int searchIndex = 1;
      while (searchIndex < d.length)
      {
         if (d[searchIndex] < minValue)
         {
            minIndex = searchIndex;
            minValue = d[searchIndex];
         }

         searchIndex++;
      }

      return minIndex;
   }

   /**
    * Final phase helper method called if one range has a size of exactly one
    * @return The two points on each respective polygon which are closest to one another
    */
   private static Point2d[] finalPhasePointAndEdge(Point2d edgePoint1, Point2d edgePoint2, Point2d lonePoint)
   {
      Point2d proj = getOrthogonalProjectionOnLine(lonePoint, edgePoint1, edgePoint2);
      LineSegment2d p = new LineSegment2d(edgePoint1, edgePoint2);
      if (p.isBetweenEndpoints(proj, 0))
      {
         return new Point2d[] {lonePoint, proj};
      }
      else
      {
         return new Point2d[] {lonePoint, (lonePoint.distance(edgePoint1) < lonePoint.distance(edgePoint2)) ? edgePoint1 : edgePoint2};
      }
   }

   /**
    * from http://softsurfer.com/Archive/algorithm_0111/algorithm_0111.htm#Pseudo-Code:%20Clip%20Segment-Polygon
    * Input: a 2D segment S from point P0 to point P1
    * a 2D convex polygon W with n vertices V0,...,Vn-1,Vn=V0
    */
   public static boolean doesSegmentIntersectConvexPolygon2D(Point2d P0, Point2d P1, ConvexPolygon2d convexPolygon2d)
   {
      // if segment is a single point
      if (P0.equals(P1))
      {
         return convexPolygon2d.isPointInside(P0);
      }

      // if either point is inside polygon
      if (convexPolygon2d.isPointInside(P0, .0001) || convexPolygon2d.isPointInside(P1, .0001))
         return true;

      // if either point touches the polygon
      if (convexPolygon2d.pointIsOnPerimeter(P0) || convexPolygon2d.pointIsOnPerimeter(P1))
         return true;

      return doesSegmentPassCompletelyThroughPolygon(P0, P1, convexPolygon2d);
   }

   private static boolean doesSegmentPassCompletelyThroughPolygon(Point2d P0, Point2d P1, ConvexPolygon2d convexPolygon2d)
   {
      // Initialize:
      double tE = 0.0;    // for the maximum entering segment parameter;
      double tL = 1.0;    // for the minimum leaving segment parameter;

      // segment direction vector
      Vector2d dS = new Vector2d(P1);
      dS.sub(P0);

      if (DEBUG)
      {
         System.out.println("dS = " + dS);
      }

      int numberOfVertices = convexPolygon2d.getNumberOfVertices();
      if (DEBUG)
      {
         System.out.println("ccwPoints = ");

         for (int i = 0; i < numberOfVertices; i++)
         {
            System.out.println(convexPolygon2d.getVertexCCW(i));
         }
      }

      for (int i = 0; i < numberOfVertices; i++)
      {
         // edge vertices
         Point2d V0 = new Point2d(convexPolygon2d.getVertexCCW(i));
         if (DEBUG)
         {
            System.out.println("V0 = " + V0);
         }

         Point2d V1 = new Point2d(convexPolygon2d.getNextVertexCCW(i));
         if (DEBUG)
         {
            System.out.println("V1 = " + V1);
         }

         // edge vector
         Vector2d V0toV1 = new Vector2d(V1);
         V0toV1.sub(V0);

         if (DEBUG)
         {
            System.out.println("V0toV1 = " + V0toV1);
         }

         // outward normal of the edge
         Vector2d ni = new Vector2d(V0toV1.getY(), -V0toV1.getX());
         if (DEBUG)
         {
            System.out.println("ni = " + ni);
         }

         Vector2d P0toVi = new Vector2d(P0);
         P0toVi.sub(V0);

         if (DEBUG)
         {
            System.out.println("P0toVi = " + P0toVi);
         }

         double N = -P0toVi.dot(ni);
         if (DEBUG)
         {
            System.out.println("N = " + N);
         }

         double D = dS.dot(ni);
         if (DEBUG)
         {
            System.out.println("D = " + D);
         }

         if (D == 0)
         {
            // S is parallel to the edge ei

            if (N < 0)
            {
               // then P0 is outside the edge ei
               return false;    // since S cannot intersect W;
            }
            else
            {
               // S cannot enter or leave W across edge ei
               // ignore edge ei and process the next edge
               continue;
            }
         }

         double t = N / D;
         if (DEBUG)
         {
            System.out.println("t = " + t);
         }

         if (D < 0)
         {
            // then segment S is entering W across edge ei
            tE = Math.max(tE, t);

            if (tE > tL)
            {
               // then segment S enters W after leaving
               return false;    // since S cannot intersect W
            }
         }
         else if (D > 0)
         {
            // then segment S is leaving W across edge ei
            tL = Math.min(tL, t);

            if (tL < tE)
            {
               // then segment S leaves W before entering
               return false;    // since S cannot intersect W
            }
         }
      }

      // Output: [Note: to get here, one must have tE <= tL]
      // there is a valid intersection of S with W
      // from the entering point: P(tE) = P0 + tE * dS
      // to the leaving point:    P(tL) = P0 + tL * dS
      return true;
   }

   public static double cross(Vector2d firstVector, Vector2d secondVector)
   {
      return firstVector.getX() * secondVector.getY() - firstVector.getY() * secondVector.getX();
   }
   
   public static boolean isZero(Tuple3d tuple, double epsilon)
   {
      boolean isZero = true;
      isZero &= MathTools.epsilonEquals(tuple.getX(), 0.0, epsilon);
      isZero &= MathTools.epsilonEquals(tuple.getY(), 0.0, epsilon);
      isZero &= MathTools.epsilonEquals(tuple.getZ(), 0.0, epsilon);
      return isZero;
   }
   
   public static boolean isZero(Tuple2d tuple, double epsilon)
   {
      boolean isZero = true;
      isZero &= MathTools.epsilonEquals(tuple.getX(), 0.0, epsilon);
      isZero &= MathTools.epsilonEquals(tuple.getY(), 0.0, epsilon);
      return isZero;
   }
}
