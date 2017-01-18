package us.ihmc.robotics.geometry;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;

import us.ihmc.robotics.geometry.transformables.TransformablePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * A line segment must have two distinct endpoints by definition.
 *
 * @author Twan Koolen
 */
public class LineSegment2d implements Geometry2d<LineSegment2d>
{
   protected TransformablePoint2d[] endpoints = new TransformablePoint2d[2];
   
   private final Point2d tempPoint2d = new Point2d();
   
   private final Vector2d tempVector2dOne = new Vector2d(); 
   private final Vector2d tempVector2dTwo = new Vector2d();

   public LineSegment2d()
   {
      endpoints[0] = new TransformablePoint2d(Double.MIN_VALUE, Double.MIN_VALUE);
      endpoints[1] = new TransformablePoint2d(Double.MAX_VALUE, Double.MAX_VALUE);
   }

   public LineSegment2d(double firstEndpointX, double firstEndpointY, double secondEndpointX, double secondEndpointY)
   {
      endpoints[0] = new TransformablePoint2d(firstEndpointX, firstEndpointY);
      endpoints[1] = new TransformablePoint2d(secondEndpointX, secondEndpointY);
      checkEndpointsDistinct(endpoints);
   }

   public LineSegment2d(Point2d[] endpoints)
   {
      checkEndpointsDistinct(endpoints);
      this.endpoints[0] = new TransformablePoint2d(endpoints[0]);
      this.endpoints[1] = new TransformablePoint2d(endpoints[1]);
   }

   public LineSegment2d(Point2d endpoint1, Point2d endpoint2)
   {
      checkEndpointsDistinct(new Point2d[] {endpoint1, endpoint2});
      endpoints[0] = new TransformablePoint2d(endpoint1);
      endpoints[1] = new TransformablePoint2d(endpoint2);
   }

   public LineSegment2d(LineSegment2d lineSegment2d)
   {
      endpoints = lineSegment2d.getEndpointsCopy();
   }

   public TransformablePoint2d[] getEndpointsCopy()
   {
      return new TransformablePoint2d[] {new TransformablePoint2d(endpoints[0]), new TransformablePoint2d(endpoints[1])};
   }

   public void getEndpoints(Point2d endpoint0, Point2d endpoint1)
   {
      endpoint0.set(endpoints[0]);
      endpoint1.set(endpoints[1]);
   }
   
   public Point2d getFirstEndpoint()
   {
      return endpoints[0];
   }
   
   public Point2d getSecondEndpoint()
   {
      return endpoints[1];
   }

   public Point2d[] getEndpoints()
   {
      return endpoints;
   }

   public Point2d getFirstEndpointCopy()
   {
      return new Point2d(endpoints[0]);
   }

   public Point2d getSecondEndpointCopy()
   {
      return new Point2d(endpoints[1]);
   }

   public double getFirstEndpointX()
   {
      return endpoints[0].getX();
   }

   public double getFirstEndpointY()
   {
      return endpoints[0].getY();
   }

   public double getSecondEndpointX()
   {
      return endpoints[1].getX();
   }

   public double getSecondEndpointY()
   {
      return endpoints[1].getY();
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

   public void set(double firstEndpointX, double firstEndpointY, double secondEndpointX, double secondEndpointY)
   {
      endpoints[0].set(firstEndpointX, firstEndpointY);
      endpoints[1].set(secondEndpointX, secondEndpointY);
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

   @Override
   public void set(LineSegment2d lineSegment)
   {
      endpoints[0].set(lineSegment.endpoints[0]);
      endpoints[1].set(lineSegment.endpoints[1]);
   }

   public void flipDirection()
   {
      double xTemp = endpoints[0].getX();
      double yTemp = endpoints[0].getY();

      endpoints[0].set(endpoints[1]);
      endpoints[1].set(xTemp, yTemp);
   }

   public Point2d midpoint()
   {
      Point2d point2d = new Point2d();
      
      getMidpoint(point2d);

      return point2d;
   }
   
   public void getMidpoint(Point2d midpointToPack)
   {
      midpointToPack.setX((endpoints[0].getX() + endpoints[1].getX()) / 2.0);
      midpointToPack.setY((endpoints[0].getY() + endpoints[1].getY()) / 2.0);
   }

   public double length()
   {
      return endpoints[0].distance(endpoints[1]);
   }

   public double dotProduct(LineSegment2d lineSegment2d)
   {
      double dotProduct = (endpoints[1].getX() - endpoints[0].getX()) * (lineSegment2d.endpoints[1].getX() - lineSegment2d.endpoints[0].getX())
                          + (endpoints[1].getY() - endpoints[0].getY()) * (lineSegment2d.endpoints[1].getY() - lineSegment2d.endpoints[0].getY());

      return dotProduct;
   }

   public boolean isBetweenEndpoints(Point2d point2d, double epsilon)
   {
      return isBetweenEndpoints(point2d.getX(), point2d.getY(), epsilon);
   }
   
   private boolean isBetweenEndpoints(double x, double y, double epsilon)
   {
      double alpha = percentageAlongLineSegment(x, y);

      if (alpha < epsilon)
         return false;
      if (alpha > 1.0 - epsilon)
         return false;

      return true;
   }

   public boolean isPointOnLeftSideOfLineSegment(Point2d point)
   {
      return GeometryTools.isPointOnSideOfLine(point, endpoints[0], endpoints[1], RobotSide.LEFT);
   }
   
   public boolean isPointOnRightSideOfLineSegment(Point2d point)
   {
      return GeometryTools.isPointOnSideOfLine(point, endpoints[0], endpoints[1], RobotSide.RIGHT);
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

      endpoints[0].setX(endpoints[0].getX() + vectorXPerpToRight);
      endpoints[0].setY(endpoints[0].getY() + vectorYPerpToRight);
      endpoints[1].setX(endpoints[1].getX() + vectorXPerpToRight);
      endpoints[1].setY(endpoints[1].getY() + vectorYPerpToRight);
   }
   
   public double percentageAlongLineSegment(Point2d point2d)
   {
      return percentageAlongLineSegment(point2d.getX(), point2d.getY());
   }

   private double percentageAlongLineSegment(double x, double y)
   {
      double vx0 = x - endpoints[0].getX();
      double vy0 = y - endpoints[0].getY();

      double vx1 = endpoints[1].getX() - endpoints[0].getX();
      double vy1 = endpoints[1].getY() - endpoints[0].getY();

      double dot = vx0 * vx1 + vy0 * vy1;
      double lengthSquared = vx1 * vx1 + vy1 * vy1;

      double alpha = dot / lengthSquared;

      return alpha;
   }

   public boolean isPointOnLineSegment(Point2d point2d)
   {
      return isPointOnLineSegment(point2d.getX(), point2d.getY());
   }
   
   private boolean isPointOnLineSegment(double x, double y)
   {
      double vx0 = x - endpoints[0].getX();
      double vy0 = y - endpoints[0].getY();

      double vx1 = endpoints[1].getX() - endpoints[0].getX();
      double vy1 = endpoints[1].getY() - endpoints[0].getY();

      double cross = vx0 * vy1 - vy0 * vx1;
      boolean pointIsOnLine = Math.abs(cross) < 1e-7;

      return pointIsOnLine && isBetweenEndpoints(x, y, 0.0);
   }

   private final double[] tempAlphaBeta = new double[2];
   
   @Override
   public Point2d intersectionWith(LineSegment2d secondLineSegment2d)
   {
      Point2d returnPoint2d = new Point2d();
      
      intersectionWith(secondLineSegment2d, returnPoint2d);

      if (Double.isNaN(returnPoint2d.getX()) || Double.isNaN(returnPoint2d.getY()))
      {
         return null;
      }
      else
      {
         return returnPoint2d;
      }
   }
   
   public void intersectionWith(LineSegment2d lineSegment, Point2d intersectionToPack)
   {
      intersectionToPack.set(intersectionWithLineSegment(lineSegment.getFirstEndpointX(), lineSegment.getFirstEndpointY(),
                                                         lineSegment.getSecondEndpointX(), lineSegment.getSecondEndpointY()));
   }
   
   private Point2d intersectionWithLineSegment(double x0, double y0, double x1, double y1)
   {
      double vx0 = endpoints[1].getX() - endpoints[0].getX();
      double vy0 = endpoints[1].getY() - endpoints[0].getY();

      double vx1 = x1 - x0;
      double vy1 = y1 - y0;

      GeometryTools.intersection(endpoints[0].getX(), endpoints[0].getY(), vx0, vy0, x0, y0, vx1, vy1, tempAlphaBeta);
      if (Double.isNaN(tempAlphaBeta[0]))
      {
         if (endpoints[0].getX() == x0 && endpoints[0].getY() == y0)
         {
            Vector2d v1 = tempVector2dOne;
            v1.set(endpoints[0].getX() - endpoints[1].getX(), endpoints[0].getY() - endpoints[1].getY());
            Vector2d v2 = tempVector2dTwo;
            v2.set(x0 - x1, y0 - y1);
            double length1 = v1.length();

            v1.add(v2);

            double length2 = v1.length();

            // if other points go in opposit directions
            // System.out.println("A " + length1 + " " + length2);
            if (length1 > length2)
            {
               tempPoint2d.set(endpoints[0].getX(), endpoints[0].getY());
               return tempPoint2d;
            }
         }
         else if (endpoints[0].getX() == x1 && endpoints[0].getY() == y1)
         {
            Vector2d v1 = tempVector2dOne;
            v1.set(endpoints[0].getX() - endpoints[1].getX(), endpoints[0].getY() - endpoints[1].getY());
            Vector2d v2 = tempVector2dTwo;
            v2.set(x1 - x0, y1 - y0);
            double length1 = v1.length();

            v1.add(v2);

            double length2 = v1.length();

            // if other points go in opposite directions
            // System.out.println("B " + length1 + " " + length2);
            if (length1 > length2)
            {
               tempPoint2d.set(endpoints[0].getX(), endpoints[0].getY());
               return tempPoint2d;
            }
         }
         else if (endpoints[1].getX() == x0 && endpoints[1].getY() == y0)
         {
            Vector2d v1 = tempVector2dOne;
            v1.set(endpoints[1].getX() - endpoints[0].getX(), endpoints[1].getY() - endpoints[0].getY());
            Vector2d v2 = tempVector2dTwo;
            v2.set(x0 - x1, y0 - y1);
            double length1 = v1.length();

            v1.add(v2);

            double length2 = v1.length();

            // if other points go in opposite directions
            // System.out.println("C " + length1 + " " + length2);
            if (length1 > length2)
            {
               tempPoint2d.set(endpoints[1].getX(), endpoints[1].getY());
               return tempPoint2d;
            }
         }
         else if (endpoints[1].getX() == x1 && endpoints[1].getY() == y1)
         {
            Vector2d v1 = tempVector2dOne;
            v1.set(endpoints[1].getX() - endpoints[0].getX(), endpoints[1].getY() - endpoints[0].getY());
            Vector2d v2 = tempVector2dTwo;
            v2.set(x1 - x0, y1 - y0);
            double length1 = v1.length();

            v1.add(v2);

            double length2 = v1.length();

            // if other points go in opposite directions
            // System.out.println("D " + length1 + " " + length2);
            if (length1 > length2)
            {
               tempPoint2d.set(endpoints[0].getX(), endpoints[0].getY());
               return tempPoint2d;
            }
         }

         // should check to see if they share a common endpoint
         // if endpoints are the same check that non matching end points point in opposite directions

         tempPoint2d.set(Double.NaN, Double.NaN);
         return tempPoint2d;
      }

      double alpha = tempAlphaBeta[0];
      double beta = tempAlphaBeta[1];

      if ((alpha < 0.0) || (alpha > 1.0))
      {
         tempPoint2d.set(Double.NaN, Double.NaN);
         return tempPoint2d;
      }
      if ((beta < 0.0) || (beta > 1.0))
      {
         tempPoint2d.set(Double.NaN, Double.NaN);
         return tempPoint2d;
      }

      tempPoint2d.set(endpoints[0].getX() + vx0 * tempAlphaBeta[0], endpoints[0].getY() + vy0 * tempAlphaBeta[0]);
      return tempPoint2d;
   }

   @Override
   public Point2d intersectionWith(Line2d line2d)
   {
      Point2d returnPoint2d =  intersectionWithLine(line2d.getPoint().getX(), line2d.getPoint().getY(),
                                                    line2d.getNormalizedVector().getX(), line2d.getNormalizedVector().getY());
      
      if (Double.isNaN(returnPoint2d.getX()) || Double.isNaN(returnPoint2d.getY()))
      {
         return null;
      }
      else
      {
         return returnPoint2d;
      }
   }
   
   private Point2d intersectionWithLine(double originX, double originY, double directionX, double directionY)
   {
      double vx1 = endpoints[1].getX() - endpoints[0].getX();
      double vy1 = endpoints[1].getY() - endpoints[0].getY();

      GeometryTools.intersection(originX, originY, directionX, directionY, endpoints[0].getX(), endpoints[0].getY(), vx1, vy1, tempAlphaBeta);
      if (Double.isNaN(tempAlphaBeta[0]))
      {
         tempPoint2d.set(Double.NaN, Double.NaN);
         return tempPoint2d;
      }

      double alpha = tempAlphaBeta[0];
      double beta = tempAlphaBeta[1];

      if ((beta < 0.0) || (beta > 1.0))
      {
         tempPoint2d.set(Double.NaN, Double.NaN);
         return tempPoint2d;
      }
      
      tempPoint2d.set(originX + directionX * alpha, originY + directionY * alpha);
      return tempPoint2d;
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

   private static void checkEndpointsDistinct(Point2d[] endpoints)
   {
      if (areEndpointsTheSame(endpoints[0], endpoints[1]))
      {
         throw new RuntimeException("Line segment must have two distinct endpoints");
      }
   }

   public static boolean areEndpointsTheSame(Point2d firstEndpoint, Point2d secondEndpoint)
   {
      return areEndpointsTheSame(firstEndpoint.getX(), firstEndpoint.getY(), secondEndpoint.getX(), secondEndpoint.getY());
   }
   
   public static boolean areEndpointsTheSame(double firstEndpointX, double firstEndpointY, double secondEndpointX, double secondEndpointY)
   {
      return (firstEndpointX == secondEndpointX) && (firstEndpointY == secondEndpointY);
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
         tempTransformedPoint.set(endpoints[0].getX(), endpoints[0].getY(), 0.0);
         transform.transform(tempTransformedPoint);
         endpoints[0].set(tempTransformedPoint.getX(), tempTransformedPoint.getY());
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

      double x = endpoints[0].getX() + (endpoints[1].getX() - endpoints[0].getX()) * parameter;
      double y = endpoints[0].getY() + (endpoints[1].getY() - endpoints[0].getY()) * parameter;

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
      transform.getRotation(tempRotation);

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
         projectedPointToPack.set(endpoints[1].getX() - endpoints[0].getX(), endpoints[1].getY() - endpoints[0].getY());
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

   public void getPerpendicularBisector(Vector2d perpendicularBisectorToPack, double bisectorLengthDesired)
   {
      double x = endpoints[0].getX() - endpoints[1].getX();
      double y = endpoints[0].getY() - endpoints[1].getY();
      
      perpendicularBisectorToPack.set(-y, x);
      perpendicularBisectorToPack.normalize();
      perpendicularBisectorToPack.scale(bisectorLengthDesired);
   }

   @Override
   public void setToZero()
   {
      endpoints[0].setToZero();
      endpoints[1].setToZero();
   }

   @Override
   public void setToNaN()
   {
      endpoints[0].setToNaN();
      endpoints[1].setToNaN();      
   }

   @Override
   public boolean containsNaN()
   {
      if (endpoints[0].containsNaN()) return true;
      if (endpoints[1].containsNaN()) return true;
      
      return false;
   }

   @Override
   public boolean epsilonEquals(LineSegment2d other, double epsilon)
   {
      if (endpoints[0].epsilonEquals(other.endpoints[0], epsilon) && endpoints[1].epsilonEquals(other.endpoints[1], epsilon)) return true;
      if (endpoints[0].epsilonEquals(other.endpoints[1], epsilon) && endpoints[1].epsilonEquals(other.endpoints[0], epsilon)) return true;

      return false;
   }
}
