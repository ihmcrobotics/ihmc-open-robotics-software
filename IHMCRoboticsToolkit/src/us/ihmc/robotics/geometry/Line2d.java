package us.ihmc.robotics.geometry;

import us.ihmc.robotics.robotSide.RobotSide;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2007</p>
 *
 * <p>Company: </p>
 *
 * @author Twan Koolen
 * @version 1.0
 */
public class Line2d implements Geometry2d
{
   // TODO: think about usage of epsilons in the methods.


   protected final Point2d point = new Point2d();
   protected final Vector2d normalizedVector = new Vector2d();
   private final static double doubleMinNormal;    // Double.MIN_NORMAL is not available in JRE 1.5

   private final Point2d tempPoint2d = new Point2d();

   static
   {
      // From the JRE 1.6 Documentation; this value is equivalent to Double.MIN_NORMAL
      doubleMinNormal = Double.longBitsToDouble(0x0010000000000000L);
   }

   private final static double minAllowableVectorPart = Math.sqrt(doubleMinNormal);

   public Line2d()
   {
      point.set(0.0, 0.0);
      normalizedVector.set(0.0, 1.0);
   }

   public Line2d(Point2d point, Vector2d vector)
   {
      this.point.set(point);
      normalizedVector.set(vector);
      checkReasonableVector(normalizedVector);
      normalizedVector.normalize();
   }

   public Line2d(Point2d firstPointOnLine, Point2d secondPointOnLine)
   {
      checkDistinctPoints(firstPointOnLine, secondPointOnLine);

      point.set(firstPointOnLine);
      normalizedVector.set(secondPointOnLine);
      normalizedVector.sub(firstPointOnLine);
      checkReasonableVector(normalizedVector);
      normalizedVector.normalize();
   }

   public Line2d(double x0, double y0, double x1, double y1)
   {
      point.set(x0, y0);
      normalizedVector.set(x1, y1);
      normalizedVector.sub(point);
      checkReasonableVector(normalizedVector);
      normalizedVector.normalize();
   }

   public void set(Point2d pointOnLine, Vector2d vectorAlongLine)
   {
      point.set(pointOnLine);
      normalizedVector.set(vectorAlongLine);
      checkReasonableVector(normalizedVector);
      normalizedVector.normalize();
   }

   public Line2d(Line2d line2d)
   {
      point.set(line2d.getPoint());
      normalizedVector.set(line2d.getNormalizedVector());
   }

   public void getPoint(Point2d pointToPack)
   {
      pointToPack.set(point);
   }

   public Point2d getPoint()
   {
      return point;
   }

   public void getNormalizedVector(Vector2d normalizedVectorToPack)
   {
      normalizedVectorToPack.set(normalizedVector);
   }

   public Vector2d getNormalizedVector()
   {
      return normalizedVector;
   }

   public void getPointAndNormalizedVector(Point2d pointToPack, Vector2d normalizedVectorToPack)
   {
      getPoint(pointToPack);
      getNormalizedVector(normalizedVectorToPack);
   }

   public void getTwoPointsOnLine(Point2d point1, Point2d point2)
   {
      point1.set(point);
      point2.add(point, normalizedVector);
   }

   public double getSlope()
   {
      if ((normalizedVector.x == 0.0) && (normalizedVector.y > 0.0))
      {
         return Double.POSITIVE_INFINITY;
      }

      if ((normalizedVector.x == 0.0) && (normalizedVector.y < 0.0))
      {
         return Double.NEGATIVE_INFINITY;
      }

      return normalizedVector.y / normalizedVector.x;
   }

   public void getPointGivenParameter(double t, Point2d pointToPack)
   {
      pointToPack.set(point);
      
      pointToPack.setX(pointToPack.getX() + t * normalizedVector.getX());
      pointToPack.setY(pointToPack.getY() + t * normalizedVector.getY());
   }
   
   public Point2d getPointGivenParameter(double t)
   {
      Point2d pointToReturn = new Point2d();
      getPointGivenParameter(t, pointToReturn);
      return pointToReturn;
   }

   public double getParameterGivenPointEpsilon(Point2d point, double epsilon)
   {
      if (!containsEpsilon(point, epsilon))
      {
         throw new RuntimeException("getParameterGivenPoint: point not part of line");
      }
      else
      {
         Vector2d difference = new Vector2d(point);
         difference.sub(this.point);

         return Math.signum(difference.dot(normalizedVector)) * difference.length();
      }
   }

   public double getXIntercept()
   {
      double parameterAtIntercept = -point.y / normalizedVector.y;

      return getPointGivenParameter(parameterAtIntercept).x;
   }

   public double getYIntercept()
   {
      double parameterAtIntercept = -point.x / normalizedVector.x;

      return getPointGivenParameter(parameterAtIntercept).y;
   }

   public boolean containsEpsilon(Point2d point, double epsilon)
   {
      // TODO: possibility to reduce code duplication by calling Geometry2dCalculator.distanceSquared
      // TODO: Refactor such that it is clear that this checks wether the point is within distance epsilon^2 from the line
      double vx1 = normalizedVector.x;
      double vy1 = normalizedVector.y;

      double vx2 = point.x - this.point.x;
      double vy2 = point.y - this.point.y;

      double dotProduct = vx1 * vx2 + vy1 * vy2;

      double length1Squared = (vx1 * vx1 + vy1 * vy1);
      double length2Squared = (vx2 * vx2 + vy2 * vy2);

      if (Math.abs(dotProduct * dotProduct - length1Squared * length2Squared) > epsilon)
      {
         return false;
      }
      else
      {
         return true;
      }
   }

   public void negateDirection()
   {
      normalizedVector.negate();
   }

   public Line2d negateDirectionCopy()
   {
      Line2d ret = new Line2d(this);
      ret.normalizedVector.negate();

      return ret;
   }

   public void setPoint2d(Point2d point2d)
   {
      point.set(point2d);
   }


   public void set(Point2d endpoint0, Point2d endpoint1)
   {
      if ((endpoint0.x == endpoint1.x) && (endpoint0.y == endpoint1.y))
      {
         throw new RuntimeException("Tried to set a line from two coincidal points.");
      }

      point.set(endpoint0);
      normalizedVector.set(endpoint1);
      normalizedVector.sub(endpoint0);
      normalizedVector.normalize();
   }

   public void set(double x0, double y0, double vx, double vy)
   {
      if ((Math.abs(vx) < minAllowableVectorPart) && (Math.abs(vy) < minAllowableVectorPart))
      {
         throw new RuntimeException("Line length must be greater than zero");
      }

      point.set(x0, y0);
      normalizedVector.set(vx, vy);
      normalizedVector.normalize();
   }

   public void set(Point2d[] endpoints)
   {
      if (endpoints.length != 2)
         throw new RuntimeException("Length of input array is not correct. Length = " + endpoints.length);
      this.set(endpoints[0], endpoints[1]);
   }

   public void set(Line2d line2d)
   {
      point.set(line2d.point);
      normalizedVector.set(line2d.normalizedVector);
   }


   public void rotate(double radians)
   {
      double vXOld = normalizedVector.x;
      @SuppressWarnings("unused") double vYOld = normalizedVector.y;

      double vXNew = Math.cos(radians) * vXOld - Math.sin(radians) * normalizedVector.y;
      double vYNew = Math.sin(radians) * vXOld + Math.cos(radians) * normalizedVector.y;

      normalizedVector.set(vXNew, vYNew);
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
      double vectorX = normalizedVector.x;
      double vectorY = normalizedVector.y;

      double vectorXPerpToRight = -vectorY;
      double vectorYPerpToRight = vectorX;

      if (!shiftToLeft)
      {
         vectorXPerpToRight = -vectorXPerpToRight;
         vectorYPerpToRight = -vectorYPerpToRight;
      }

      vectorXPerpToRight = distanceToShift * vectorXPerpToRight;
      vectorYPerpToRight = distanceToShift * vectorYPerpToRight;

      point.x += vectorXPerpToRight;
      point.y += vectorYPerpToRight;
   }

   public Line2d interiorBisector(Line2d secondLine)
   {
      Point2d pointOnLine = intersectionWith(secondLine);
      if (pointOnLine == null)
      {
         double distanceBetweenLines = secondLine.distance(point);
         double epsilon = 1E-7;

         boolean sameLines = distanceBetweenLines < epsilon;
         if (sameLines)
         {
            return new Line2d(this);
         }
         else
         {
            return null;
         }
      }

      Vector2d directionVector = new Vector2d(normalizedVector);
      directionVector.add(secondLine.normalizedVector);

      return new Line2d(pointOnLine, directionVector);

   }

   public void perpendicularVector(Vector2d vectorToPack)
   {
      vectorToPack.set(normalizedVector.y, -normalizedVector.x);
   }
   
   public Vector2d perpendicularVector()
   {
      Vector2d vectorToReturn = new Vector2d();
      perpendicularVector(vectorToReturn);
      return vectorToReturn;
   }

   public Line2d perpendicularLineThroughPoint(Point2d point)
   {
      return new Line2d(point, perpendicularVector());
   }

   @Override
   public Point2d intersectionWith(LineSegment2d lineSegment)
   {
      return lineSegment.intersectionWith(this);
   }

   public boolean areLinesPerpendicular(Line2d line)
   {
      // Dot product of two vectors is zero if the vectors are perpendicular
      return normalizedVector.dot(line.getNormalizedVector()) < 1e-7;
   }

   private final double[] tempAlphaBeta = new double[2];
   @Override
   public Point2d intersectionWith(Line2d secondLine)
   {
      double x0 = point.x;
      double y0 = point.y;

      double vx0 = normalizedVector.x;
      double vy0 = normalizedVector.y;

      double x1 = secondLine.point.x;
      double y1 = secondLine.point.y;

      double vx1 = secondLine.normalizedVector.x;
      double vy1 = secondLine.normalizedVector.y;

      GeometryTools.intersection(x0, y0, vx0, vy0, x1, y1, vx1, vy1, tempAlphaBeta);
      if (Double.isNaN(tempAlphaBeta[0]))
         return null;

      Point2d returnPoint2d = new Point2d(x0 + vx0 * tempAlphaBeta[0], y0 + vy0 * tempAlphaBeta[0]);

      return returnPoint2d;
   }

   public boolean intersectionWith(Point2d intersectionToPack, Line2d secondLine)
   {
      double x0 = point.x;
      double y0 = point.y;

      double vx0 = normalizedVector.x;
      double vy0 = normalizedVector.y;

      double x1 = secondLine.point.x;
      double y1 = secondLine.point.y;

      double vx1 = secondLine.normalizedVector.x;
      double vy1 = secondLine.normalizedVector.y;

      GeometryTools.intersection(x0, y0, vx0, vy0, x1, y1, vx1, vy1, tempAlphaBeta);
      if (Double.isNaN(tempAlphaBeta[0]))
         return false;

      intersectionToPack.set(x0 + vx0 * tempAlphaBeta[0], y0 + vy0 * tempAlphaBeta[0]);

      return true;
   }

   @Override
   public Point2d[] intersectionWith(ConvexPolygon2d convexPolygon)
   {
      return convexPolygon.intersectionWith(this);
   }

   @Override
   public double distance(Point2d point)
   {
      tempPoint2d.set(this.point);
      tempPoint2d.add(normalizedVector);
      
      return GeometryTools.distanceFromPointToLine(point, this.point, tempPoint2d);
   }

   public double distanceSquared(Point2d point)
   {
      double distance = distance(point);

      return distance * distance;
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

   @Override
   public String toString()
   {
      String ret = "";

      ret = ret + point + ", " + normalizedVector;

      return ret;
   }

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      checkIsTransformationInPlane(transform);
      applyTransformAndProjectToXYPlane(transform);
   }

   @Override
   public void applyTransformAndProjectToXYPlane(RigidBodyTransform transform)
   {
      Point3d resultPoint = new Point3d(point.x, point.y, 0.0);
      transform.transform(resultPoint);

      Vector3d resultVector = new Vector3d(normalizedVector.x, normalizedVector.y, 0.0);
      transform.transform(resultVector);

      point.set(resultPoint.x, resultPoint.y);
      normalizedVector.set(resultVector.x, resultVector.y);
   }

   @Override
   public Line2d applyTransformCopy(RigidBodyTransform transform)
   {
      Line2d copy = new Line2d(this);
      copy.applyTransform(transform);
      return copy;
   }

   @Override
   public Line2d applyTransformAndProjectToXYPlaneCopy(RigidBodyTransform transform)
   {
      Line2d copy = new Line2d(this);
      copy.applyTransformAndProjectToXYPlane(transform);
      return copy;
   }

   public boolean isPointOnLine(Point2d point)
   {
      // y = A*x + b with point = (x,y)
      double A = normalizedVector.getY()/normalizedVector.getX();
      double b = this.point.getY()-A*this.point.getX();

      double value = point.getY() - A*point.getX() - b;

      return 1e-8 > value;
   }

   public boolean isPointOnLeftSideOfLine(Point2d point)
   {
      return isPointOnSideOfLine(point, RobotSide.LEFT);
   }

   public boolean isPointOnRightSideOfLine(Point2d point)
   {
      return isPointOnSideOfLine(point, RobotSide.RIGHT);
   }

   public boolean isPointOnSideOfLine(Point2d point, RobotSide side)
   {
      double vectorX = normalizedVector.x;
      double vectorY = normalizedVector.y;
      double pointToPointX = point.x - this.point.x;
      double pointToPointY = point.y - this.point.y;

      double crossProduct = vectorX * pointToPointY - pointToPointX * vectorY;
      return side.negateIfRightSide(crossProduct) > 0.0;
   }

   /**
    * isPointInFrontOfLine returns whether the point is in front of the line or
    * not. The front direction is defined as the positive x-direction
    *
    * @param point Point2d
    * @return boolean
    */
   public boolean isPointInFrontOfLine(Point2d point)
   {
      if (normalizedVector.y > 0.0)
         return isPointOnRightSideOfLine(point);
      else if (normalizedVector.y < 0.0)
         return isPointOnLeftSideOfLine(point);
      else
         throw new RuntimeException("Not defined when line is pointing exactly along the x-axis");
   }

   // TODO: Inconsistency in strictness.
   public boolean isPointBehindLine(Point2d point)
   {
      return !isPointInFrontOfLine(point);
   }

   public void setParallelLineThroughPoint(Point2d point)
   {
      this.point.set(point);
   }

   public boolean containsNaN()
   {
      if (Double.isNaN(point.x))
         return true;
      if (Double.isNaN(point.y))
         return true;
      if (Double.isNaN(normalizedVector.x))
         return true;
      if (Double.isNaN(normalizedVector.y))
         return true;

      return false;
   }

   private boolean isTransformationInPlane(RigidBodyTransform transform)
   {
      // arguably not a sufficient condition. Reference frames could just happen to be aligned (not likely though). ReferenceFrame2d needed!
      double[] transformArray = new double[16];
      transform.get(transformArray);

      if (((transformArray[2] == 0.0) && (transformArray[6] == 0.0) && (transformArray[8] == 0.0) && (transformArray[9] == 0.0) && (transformArray[10] == 1.0)))
      {
         return true;
      }
      else
      {
         return false;
      }
   }

   private void checkIsTransformationInPlane(RigidBodyTransform transform)
   {
      if (!isTransformationInPlane(transform))
      {
         throw new RuntimeException("Cannot transform FrameLine2d to a plane with a different surface normal");
      }
   }


   /**
    * Compute the orthogonal projection of the given point and modify it to store the result.
    */
   @Override
   public void orthogonalProjection(Point2d point2d)
   {
//    Point2d[] endPoints = lineSegment.endpoints;
      Point2d endPoint = point;

      double vx0 = point2d.x - endPoint.x;
      double vy0 = point2d.y - endPoint.y;

      double vx1 = normalizedVector.x;
      double vy1 = normalizedVector.y;

      double dot = vx0 * vx1 + vy0 * vy1;
      double lengthSquared = vx1 * vx1 + vy1 * vy1;

      double alpha = dot / lengthSquared;

//    if (alpha < 0.0) alpha = 0.0;
//    if (alpha > 1.0) alpha = 1.0;

      double x = endPoint.x + alpha * vx1;
      double y = endPoint.y + alpha * vy1;

      point2d.x = x;
      point2d.y = y;
   }

// TODO move to Line2d
   @Override
   public Point2d orthogonalProjectionCopy(Point2d point)
   {
      Point2d copy = new Point2d(point);
      orthogonalProjection(copy);

      return copy;
   }

   public boolean equals(Line2d otherLine)
   {
      return point.equals(otherLine.point) && normalizedVector.equals(otherLine.normalizedVector);
   }

   private void checkReasonableVector(Vector2d localVector)
   {
      if ((Math.abs(localVector.x) < minAllowableVectorPart) && (Math.abs(localVector.y) < minAllowableVectorPart))
      {
         throw new RuntimeException("Line length must be greater than zero");
      }
   }

   private void checkDistinctPoints(Point2d firstPointOnLine, Point2d secondPointOnLine)
   {
      if ((firstPointOnLine.x == secondPointOnLine.x) && (firstPointOnLine.y == secondPointOnLine.y))
      {
         throw new RuntimeException("Tried to create a line from two coincidal points");
      }
   }
}
