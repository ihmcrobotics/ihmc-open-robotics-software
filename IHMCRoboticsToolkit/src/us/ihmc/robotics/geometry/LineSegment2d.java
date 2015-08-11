package us.ihmc.robotics.geometry;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;

/**
 * <p>Title: </p>
 *
 * <p>Description: a line segment must have two distinct endpoints by definition</p>
 *
 * <p>Copyright: Copyright (c) 2007</p>
 *
 * <p>Company: </p>
 *
 * @author Twan Koolen
 * @version 1.0
 */
public class LineSegment2d implements Geometry2d
{
   protected Point2d[] endpoints = new Point2d[2];

   public LineSegment2d()
   {
      endpoints[0] = new Point2d(Double.MIN_VALUE, Double.MIN_VALUE);
      endpoints[1] = new Point2d(Double.MAX_VALUE, Double.MAX_VALUE);
   }

   public LineSegment2d(double x0, double y0, double x1, double y1)
   {
      endpoints[0] = new Point2d(x0, y0);
      endpoints[1] = new Point2d(x1, y1);
      checkEndpointsDistinct(endpoints);
   }

   public LineSegment2d(Point2d[] endpoints)
   {
      checkEndpointsDistinct(endpoints);
      this.endpoints[0] = new Point2d(endpoints[0]);
      this.endpoints[1] = new Point2d(endpoints[1]);
   }

   public LineSegment2d(Point2d endpoint1, Point2d endpoint2)
   {
      checkEndpointsDistinct(new Point2d[] {endpoint1, endpoint2});
      endpoints[0] = new Point2d(endpoint1);
      endpoints[1] = new Point2d(endpoint2);
   }

   public LineSegment2d(LineSegment2d lineSegment2d)
   {
      endpoints = lineSegment2d.getEndpointsCopy();
   }

   public Point2d[] getEndpointsCopy()
   {
      return new Point2d[] {new Point2d(endpoints[0]), new Point2d(endpoints[1])};
   }

   public void getEndpoints(Point2d endpoint0, Point2d endpoint1)
   {
      endpoint0.set(endpoints[0]);
      endpoint1.set(endpoints[1]);
   }

   public Point2d[] getEndpoints()
   {
      return endpoints;
   }

   public Point2d getFirstEndPointCopy()
   {
      return new Point2d(endpoints[0]);
   }

   public Point2d getSecondEndPointCopy()
   {
      return new Point2d(endpoints[1]);
   }

   public double getX0()
   {
      return endpoints[0].x;
   }

   public double getY0()
   {
      return endpoints[0].y;
   }

   public double getX1()
   {
      return endpoints[1].x;
   }

   public double getY1()
   {
      return endpoints[1].y;
   }

   public void set(Point2d endpoint0, Point2d endpoint1)
   {
      endpoints[0].set(endpoint0);
      endpoints[1].set(endpoint1);
      checkEndpointsDistinct(endpoints);
   }

   public void set(Point2d endpoint0, Vector2d fromPoint0ToPoint1)
   {
      endpoints[0].set(endpoint0);
      endpoints[1].set(endpoint0);
      endpoints[1].add(fromPoint0ToPoint1);
      checkEndpointsDistinct(endpoints);
   }

   public void set(double x0, double y0, double x1, double y1)
   {
      endpoints[0].set(x0, y0);
      endpoints[1].set(x1, y1);
      checkEndpointsDistinct(endpoints);
   }

   public void set(Point2d[] endpoints)
   {
      if (endpoints.length != 2)
         throw new RuntimeException("Length of input array is not correct. Length = " + endpoints.length);
      this.endpoints[0].set(endpoints[0]);
      this.endpoints[1].set(endpoints[1]);
      checkEndpointsDistinct(endpoints);

   }

   public void set(LineSegment2d lineSegment)
   {
      endpoints[0].set(lineSegment.endpoints[0]);
      endpoints[1].set(lineSegment.endpoints[1]);
   }

   public void flipDirection()
   {
      double xTemp = endpoints[0].x;
      double yTemp = endpoints[0].y;

      endpoints[0].set(endpoints[1]);
      endpoints[1].set(xTemp, yTemp);
   }

   public Point2d midpoint()
   {
      double x = (endpoints[0].x + endpoints[1].x) / 2.0;
      double y = (endpoints[0].y + endpoints[1].y) / 2.0;

      return new Point2d(x, y);
   }

   public double length()
   {
      return endpoints[0].distance(endpoints[1]);
   }

   public double dotProduct(LineSegment2d lineSegment2d)
   {
      double dotProduct = (endpoints[1].x - endpoints[0].x) * (lineSegment2d.endpoints[1].x - lineSegment2d.endpoints[0].x)
                          + (endpoints[1].y - endpoints[0].y) * (lineSegment2d.endpoints[1].y - lineSegment2d.endpoints[0].y);

      return dotProduct;
   }


   public boolean isBetweenEndpoints(Point2d point2d, double epsilon)
   {
      double alpha = percentageAlongLineSegment(point2d);

      if (alpha < epsilon)
         return false;
      if (alpha > 1.0 - epsilon)
         return false;

      return true;
   }

   public boolean isPointOnLeftSideOfLineSegment(Point2d point)
   {
      double vectorX = endpoints[1].getX() - endpoints[0].getX();
      double vectorY = endpoints[1].getY() - endpoints[0].getY();
      double pointToPointX = point.x - endpoints[0].getX();
      double pointToPointY = point.y - endpoints[0].getY();

      double crossProduct = vectorX * pointToPointY - pointToPointX * vectorY;
      if (crossProduct > 0.0)
      {
         return true;
      }
      else
      {
         return false;
      }
   }

   public boolean isPointOnRightSideOfLineSegment(Point2d point)    // also returns true if the point is on the line
   {
      return !isPointOnLeftSideOfLineSegment(point);
   }

   public LineSegment2d shiftToLeftCopy(double distanceToShift)
   {
      return shiftAndCopy(true, distanceToShift);
   }

   public LineSegment2d shiftToRightCopy(double distanceToShift)
   {
      return shiftAndCopy(false, distanceToShift);
   }

   private LineSegment2d shiftAndCopy(boolean shiftToLeft, double distanceToShift)
   {
      double vectorX = endpoints[1].getX() - endpoints[0].getX();
      double vectorY = endpoints[1].getY() - endpoints[0].getY();

      double vectorXPerpToRight = -vectorY;
      double vectorYPerpToRight = vectorX;

      if (!shiftToLeft)
      {
         vectorXPerpToRight = -vectorXPerpToRight;
         vectorYPerpToRight = -vectorYPerpToRight;
      }

      double vectorPerpToRightLength = Math.sqrt(vectorXPerpToRight * vectorXPerpToRight + vectorYPerpToRight * vectorYPerpToRight);
      vectorXPerpToRight = distanceToShift * vectorXPerpToRight / vectorPerpToRightLength;
      vectorYPerpToRight = distanceToShift * vectorYPerpToRight / vectorPerpToRightLength;

      Point2d newEndpoint0 = new Point2d(endpoints[0].getX() + vectorXPerpToRight, endpoints[0].getY() + vectorYPerpToRight);
      Point2d newEndpoint1 = new Point2d(endpoints[1].getX() + vectorXPerpToRight, endpoints[1].getY() + vectorYPerpToRight);

      LineSegment2d ret = new LineSegment2d(newEndpoint0, newEndpoint1);

      return ret;
   }

   public void shiftToLeft(double distanceToShift)
   {
      shift(true, distanceToShift);
   }

   public void shiftToRight(double distanceToShift)
   {
      shift(false, distanceToShift);
   }

   private void shift(boolean shiftToLeft, double distanceToShift)
   {
      double vectorX = endpoints[1].getX() - endpoints[0].getX();
      double vectorY = endpoints[1].getY() - endpoints[0].getY();

      double vectorXPerpToRight = -vectorY;
      double vectorYPerpToRight = vectorX;

      if (!shiftToLeft)
      {
         vectorXPerpToRight = -vectorXPerpToRight;
         vectorYPerpToRight = -vectorYPerpToRight;
      }

      double vectorPerpToRightLength = Math.sqrt(vectorXPerpToRight * vectorXPerpToRight + vectorYPerpToRight * vectorYPerpToRight);
      vectorXPerpToRight = distanceToShift * vectorXPerpToRight / vectorPerpToRightLength;
      vectorYPerpToRight = distanceToShift * vectorYPerpToRight / vectorPerpToRightLength;

      endpoints[0].x += vectorXPerpToRight;
      endpoints[0].y += vectorYPerpToRight;
      endpoints[1].x += vectorXPerpToRight;
      endpoints[1].y += vectorYPerpToRight;
   }

   public double percentageAlongLineSegment(Point2d point2d)
   {
      double vx0 = point2d.x - endpoints[0].x;
      double vy0 = point2d.y - endpoints[0].y;

      double vx1 = endpoints[1].x - endpoints[0].x;
      double vy1 = endpoints[1].y - endpoints[0].y;

      double dot = vx0 * vx1 + vy0 * vy1;
      double lengthSquared = vx1 * vx1 + vy1 * vy1;

      double alpha = dot / lengthSquared;

      return alpha;
   }

   public boolean isPointOnLineSegment(Point2d point2d)
   {
      double vx0 = point2d.x - endpoints[0].x;
      double vy0 = point2d.y - endpoints[0].y;

      double vx1 = endpoints[1].x - endpoints[0].x;
      double vy1 = endpoints[1].y - endpoints[0].y;

      double cross = vx0 * vy1 - vy0 * vx1;
      boolean pointIsOnLine = Math.abs(cross) < 1e-7;

      return pointIsOnLine && isBetweenEndpoints(point2d, 0.0);
   }

// public boolean containsEpsilon(Point2d point, double epsilon)
// {
//    // Dot product squared should be the product of the lengths and the vector to the point should be shorter than the segment length,
//
//    double vx1 = endpoints[1].x - endpoints[0].x;
//    double vy1 = endpoints[1].y - endpoints[0].y;
//
//    double vx2 = point.x - endpoints[0].x;
//    double vy2 = point.y - endpoints[0].y;
//
//    double dotProduct = vx1 * vx2 + vy1 * vy2;
//
//    double length1Squared = (vx1 * vx1 + vy1 * vy1);
//    double length2Squared = (vx2 * vx2 + vy2 * vy2);
//
//    if (dotProduct < epsilon) return false;
//
//    if (dotProduct*dotProduct > length1Squared * length2Squared -epsilon) return false;
//
//    if (length2Squared > length1Squared - epsilon) return false;
//
//    return true;

//    // 1. Cross product should be zero-ish (point is on the line through the endpoints):
//    double crossProduct = (endpoints[1].x - endpoints[0].x) * (point.y - endpoints[0].y) -
//        (point.x - endpoints[0].x) * (endpoints[1].y - endpoints[0].y);
//    if (Math.abs(crossProduct) > epsilon)
//    {
//       return false;
//    }
//
//    // 2. Projection should be between the endpoints:
//    double dotProduct = (endpoints[1].x - endpoints[0].x) * (point.x - endpoints[0].x) +
//        (endpoints[1].y - endpoints[0].y) * (point.y - endpoints[0].y);
//    double length1 = (endpoints[1].x - endpoints[0].x) * (endpoints[1].x - endpoints[0].x) + (endpoints[1].y - endpoints[0].y) * (endpoints[1].y - endpoints[0].y);
//    double length2 = (point.x - endpoints[0].x) * (point.x - endpoints[0].x) + (point.y - endpoints[0].y) * (point.y - endpoints[0].y);
//    double projectionLength = dotProduct / (length1 * length2);
//    if (projectionLength > length1 || projectionLength < 0)
//    {
//       return false;
//    }
//
//    return true;
// }

   private final double[] tempAlphaBeta = new double[2];
   
   @Override
   public Point2d intersectionWith(LineSegment2d secondLineSegment2d)
   {
      Point2d[] endPoints0 = endpoints;

      double x0 = endPoints0[0].x;
      double y0 = endPoints0[0].y;

      double vx0 = endPoints0[1].x - x0;
      double vy0 = endPoints0[1].y - y0;

      Point2d[] endPoints1 = secondLineSegment2d.endpoints;

      double x1 = endPoints1[0].x;
      double y1 = endPoints1[0].y;

      double vx1 = endPoints1[1].x - x1;
      double vy1 = endPoints1[1].y - y1;

      GeometryTools.intersection(x0, y0, vx0, vy0, x1, y1, vx1, vy1, tempAlphaBeta);
      if (Double.isNaN(tempAlphaBeta[0]))
      {
         if (endPoints0[0].equals(endPoints1[0]))
         {
            Vector2d v1 = new Vector2d(endPoints0[0].x - endPoints0[1].x, endPoints0[0].y - endPoints0[1].y);
            Vector2d v2 = new Vector2d(endPoints1[0].x - endPoints1[1].x, endPoints1[0].y - endPoints1[1].y);
            double length1 = v1.length();

            v1.add(v2);

            double length2 = v1.length();

            // if other points go in opposit directions
            // System.out.println("A " + length1 + " " + length2);
            if (length1 > length2)
               return endPoints0[0];
         }
         else if (endPoints0[0].equals(endPoints1[1]))
         {
            Vector2d v1 = new Vector2d(endPoints0[0].x - endPoints0[1].x, endPoints0[0].y - endPoints0[1].y);
            Vector2d v2 = new Vector2d(endPoints1[1].x - endPoints1[0].x, endPoints1[1].y - endPoints1[0].y);
            double length1 = v1.length();

            v1.add(v2);

            double length2 = v1.length();

            // if other points go in opposit directions
            // System.out.println("B " + length1 + " " + length2);
            if (length1 > length2)
               return endPoints0[0];
         }

         else if (endPoints0[1].equals(endPoints1[0]))
         {
            Vector2d v1 = new Vector2d(endPoints0[1].x - endPoints0[0].x, endPoints0[1].y - endPoints0[0].y);
            Vector2d v2 = new Vector2d(endPoints1[0].x - endPoints1[1].x, endPoints1[0].y - endPoints1[1].y);
            double length1 = v1.length();

            v1.add(v2);

            double length2 = v1.length();

            // if other points go in opposit directions
            // System.out.println("C " + length1 + " " + length2);
            if (length1 > length2)
               return endPoints0[1];
         }
         else if (endPoints0[1].equals(endPoints1[1]))
         {
            Vector2d v1 = new Vector2d(endPoints0[1].x - endPoints0[0].x, endPoints0[1].y - endPoints0[0].y);
            Vector2d v2 = new Vector2d(endPoints1[1].x - endPoints1[0].x, endPoints1[1].y - endPoints1[0].y);
            double length1 = v1.length();

            v1.add(v2);

            double length2 = v1.length();

            // if other points go in opposit directions
            // System.out.println("D " + length1 + " " + length2);
            if (length1 > length2)
               return endPoints0[0];
         }

         // should check to see if they shair a common endpoint
         // if endpoints are the same check that non matching end points point in opposit directions

         return null;
      }

      double alpha = tempAlphaBeta[0];
      double beta = tempAlphaBeta[1];

      if ((alpha < 0.0) || (alpha > 1.0))
         return null;
      if ((beta < 0.0) || (beta > 1.0))
         return null;

      Point2d returnPoint2d = new Point2d(x0 + vx0 * tempAlphaBeta[0], y0 + vy0 * tempAlphaBeta[0]);

      return returnPoint2d;
   }

   @Override
   public Point2d intersectionWith(Line2d line2d)
   {
      double x0 = line2d.point.x;
      double y0 = line2d.point.y;

      double vx0 = line2d.normalizedVector.x;
      double vy0 = line2d.normalizedVector.y;

      Point2d[] endPoints1 = endpoints;

      double x1 = endPoints1[0].x;
      double y1 = endPoints1[0].y;

      double vx1 = endPoints1[1].x - x1;
      double vy1 = endPoints1[1].y - y1;

      GeometryTools.intersection(x0, y0, vx0, vy0, x1, y1, vx1, vy1, tempAlphaBeta);
      if (Double.isNaN(tempAlphaBeta[0]))
         return null;

      double alpha = tempAlphaBeta[0];
      double beta = tempAlphaBeta[1];

      if ((beta < 0.0) || (beta > 1.0))
         return null;

      Point2d returnPoint2d = new Point2d(x0 + vx0 * alpha, y0 + vy0 * alpha);

      return returnPoint2d;
   }

   @Override
   public Point2d[] intersectionWith(ConvexPolygon2d convexPolygon)
   {
      return ConvexPolygonTools.intersection(this, convexPolygon);
   }

   @Override
   public double distance(Line2d line)
   {
      throw new RuntimeException("Not yet implemented");
   }

   @Override
   public double distance(LineSegment2d lineSegment)
   {
      throw new RuntimeException("Not yet implemented");
   }

   @Override
   public double distance(ConvexPolygon2d convexPolygon)
   {
      throw new RuntimeException("Not yet implemented");
   }

   private void checkEndpointsDistinct(Point2d[] endpoints)
   {
      if ((endpoints[0].x == endpoints[1].x) && (endpoints[0].y == endpoints[1].y))
      {
         throw new RuntimeException("Line segment must have two distinct endpoints");
      }
   }

   @Override
   public String toString()
   {
      return "" + endpoints[0] + endpoints[1];
   }

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      checkIsTransformationInPlane(transform);
      applyTransformAndProjectToXYPlane(transform);
   }

   private Point3d tempTransformedPoint;

   @Override
   public void applyTransformAndProjectToXYPlane(RigidBodyTransform transform)
   {
      if (tempTransformedPoint == null)
         tempTransformedPoint = new Point3d();

      for (int i = 0; i < endpoints.length; i++)
      {
         tempTransformedPoint.set(endpoints[0].x, endpoints[0].y, 0.0);
         transform.transform(tempTransformedPoint);
         endpoints[0].set(tempTransformedPoint.x, tempTransformedPoint.y);
      }
   }

   @Override
   public LineSegment2d applyTransformCopy(RigidBodyTransform transform)
   {
      LineSegment2d copy = new LineSegment2d(this);
      copy.applyTransform(transform);
      return copy;
   }

   @Override
   public LineSegment2d applyTransformAndProjectToXYPlaneCopy(RigidBodyTransform transform)
   {
      LineSegment2d copy = new LineSegment2d(this);
      copy.applyTransformAndProjectToXYPlane(transform);
      return copy;
   }

   public void pointBetweenEndPointsGivenParameter(double parameter, Point2d pointToPack)
   {
      if ((parameter > 1.0) || (parameter < 0.0))
      {
         throw new RuntimeException("Parameter out of range. Parameter = " + parameter);
      }

      double x = endpoints[0].x + (endpoints[1].x - endpoints[0].x) * parameter;
      double y = endpoints[0].y + (endpoints[1].y - endpoints[0].y) * parameter;

      pointToPack.set(x, y);
   }
   
   public Point2d pointBetweenEndPointsGivenParameter(double parameter)
   {
      Point2d pointToReturn = new Point2d();
      
      pointBetweenEndPointsGivenParameter(parameter, pointToReturn);
      return pointToReturn;
   }

   private final Matrix3d tempRotation = new Matrix3d();

   private boolean isTransformationInPlane(RigidBodyTransform transform)
   {
      // arguably not a sufficient condition. ReferenceFrame2d needed!
      transform.get(tempRotation);

      return ReferenceFrame.isRotationInPlane(tempRotation);
   }

   private void checkIsTransformationInPlane(RigidBodyTransform transform)
   {
      if (!isTransformationInPlane(transform))
      {
         throw new RuntimeException("Cannot transform FrameLineSegment2d to a plane with a different surface normal");
      }
   }

   /**
    * Compute the smallest distance from the point to this line segment.
    * If the projection of the given point on this line segment results in a point that is outside the line segment, the distance is computed between the given point and the closest line segment end point.
    */
   @Override
   public double distance(Point2d point)
   {
      double alpha = percentageAlongLineSegment(point);

      if (alpha <= 0.0)
      {
         return point.distance(endpoints[0]);
      }
      else if (alpha >= 1.0)
      {
         return point.distance(endpoints[1]);
      }
      else
      {
         // Here we know the projection of the point belongs to the line segment.
         // In this case computing the distance from the point to the line segment is the same as computing the distance from the point the equivalent line.
         return GeometryTools.distanceFromPointToLine(point, endpoints[0], endpoints[1]);
      }
   }

// TODO move to LineSegment
   @Override
   public Point2d orthogonalProjectionCopy(Point2d point)
   {
      Point2d copy = new Point2d(point);
      orthogonalProjection(copy);

      return copy;
   }

   /**
    * Compute the orthogonal projection of the given point and modify it to store the result.
    * If the projection results in a point that is not in the 
    */
   @Override
   public void orthogonalProjection(Point2d point2d)
   {
      orthogonalProjection(point2d, point2d);
   }

   public void orthogonalProjection(Point2d projectedPointToPack, Point2d point2d)
   {
      double alpha = percentageAlongLineSegment(point2d);

      if (alpha <= 0.0)
      {
         projectedPointToPack.set(endpoints[0]);
      }
      else if (alpha >= 1.0)
      {
         projectedPointToPack.set(endpoints[1]);
      }
      else
      {
         projectedPointToPack.set(endpoints[1].x - endpoints[0].x, endpoints[1].y - endpoints[0].y);
         projectedPointToPack.scale(alpha);
         projectedPointToPack.add(endpoints[0]);
      }
   }

   public Point2d getClosestPointOnLineSegmentCopy(Point2d point2d)
   {
      Point2d closestPointToReturn = new Point2d();
      getClosestPointOnLineSegment(closestPointToReturn, point2d);
      return closestPointToReturn;
   }

   public void getClosestPointOnLineSegment(Point2d closestPointToPack, Point2d point2d)
   {
      orthogonalProjection(closestPointToPack, point2d);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == null) return false;
      if (!(object instanceof LineSegment2d)) return false;
      if (object == this) return true;

      LineSegment2d otherSegment = (LineSegment2d) object;

      if (endpoints[0].equals(otherSegment.endpoints[0]) && endpoints[1].equals(otherSegment.endpoints[1]))
         return true;
      else if (endpoints[0].equals(otherSegment.endpoints[1]) && endpoints[1].equals(otherSegment.endpoints[0]))
         return true;

      return false;
   }
}
