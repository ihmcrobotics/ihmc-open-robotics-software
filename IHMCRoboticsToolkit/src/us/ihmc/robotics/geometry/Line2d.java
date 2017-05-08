package us.ihmc.robotics.geometry;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * Represents an infinitely-long 2D line defined by a 2D point and a 2D unit-vector.
 */
public class Line2d implements GeometryObject<Line2d>
{
   private final static double minAllowableVectorPart = Math.sqrt(Double.MIN_NORMAL);

   /** Coordinates of a point located on this line. */
   protected final Point2D point = new Point2D();
   /** Normalized direction of this line. */
   protected final Vector2D direction = new Vector2D();

   public Line2d()
   {
      point.set(0.0, 0.0);
      direction.set(0.0, 1.0);
   }

   public Line2d(Point2DReadOnly point, Vector2DReadOnly vector)
   {
      this.point.set(point);
      direction.set(vector);
      checkReasonableVector(direction);
      direction.normalize();
   }

   public Line2d(Point2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine)
   {
      checkDistinctPoints(firstPointOnLine, secondPointOnLine);

      point.set(firstPointOnLine);
      direction.set(secondPointOnLine);
      direction.sub(firstPointOnLine);
      checkReasonableVector(direction);
      direction.normalize();
   }

   public Line2d(double x0, double y0, double x1, double y1)
   {
      point.set(x0, y0);
      direction.set(x1, y1);
      direction.sub(point);
      checkReasonableVector(direction);
      direction.normalize();
   }

   public void set(Point2DReadOnly pointOnLine, Vector2DReadOnly vectorAlongLine)
   {
      point.set(pointOnLine);
      direction.set(vectorAlongLine);
      checkReasonableVector(direction);
      direction.normalize();
   }

   public Line2d(Line2d line2d)
   {
      point.set(line2d.getPoint());
      direction.set(line2d.getNormalizedVector());
   }

   public void getPoint(Point2DBasics pointToPack)
   {
      pointToPack.set(point);
   }

   public Point2D getPoint()
   {
      return point;
   }

   public void getNormalizedVector(Vector2DBasics normalizedVectorToPack)
   {
      normalizedVectorToPack.set(direction);
   }

   public Vector2D getNormalizedVector()
   {
      return direction;
   }

   public void getPointAndNormalizedVector(Point2DBasics pointToPack, Vector2DBasics normalizedVectorToPack)
   {
      getPoint(pointToPack);
      getNormalizedVector(normalizedVectorToPack);
   }

   public void getTwoPointsOnLine(Point2DBasics point1, Point2DBasics point2)
   {
      point1.set(point);
      point2.add(point, direction);
   }

   public double getSlope()
   {
      if ((direction.getX() == 0.0) && (direction.getY() > 0.0))
      {
         return Double.POSITIVE_INFINITY;
      }

      if ((direction.getX() == 0.0) && (direction.getY() < 0.0))
      {
         return Double.NEGATIVE_INFINITY;
      }

      return direction.getY() / direction.getX();
   }

   public void getPointGivenParameter(double t, Point2DBasics pointToPack)
   {
      pointToPack.set(point);

      pointToPack.setX(pointToPack.getX() + t * direction.getX());
      pointToPack.setY(pointToPack.getY() + t * direction.getY());
   }

   public Point2D getPointGivenParameter(double t)
   {
      Point2D pointToReturn = new Point2D();
      getPointGivenParameter(t, pointToReturn);
      return pointToReturn;
   }

   public double getParameterGivenPointEpsilon(Point2DReadOnly point, double epsilon)
   {
      if (!containsEpsilon(point, epsilon))
      {
         throw new RuntimeException("getParameterGivenPoint: point not part of line");
      }
      else
      {
         Vector2D difference = new Vector2D(point);
         difference.sub(this.point);

         return Math.signum(difference.dot(direction)) * difference.length();
      }
   }

   public double getXIntercept()
   {
      double parameterAtIntercept = -point.getY() / direction.getY();

      return getPointGivenParameter(parameterAtIntercept).getX();
   }

   public double getYIntercept()
   {
      double parameterAtIntercept = -point.getX() / direction.getX();

      return getPointGivenParameter(parameterAtIntercept).getY();
   }

   public boolean containsEpsilon(Point2DReadOnly point, double epsilon)
   {
      // TODO: possibility to reduce code duplication by calling Geometry2dCalculator.distanceSquared
      // TODO: Refactor such that it is clear that this checks wether the point is within distance epsilon^2 from the line
      double vx1 = direction.getX();
      double vy1 = direction.getY();

      double vx2 = point.getX() - this.point.getX();
      double vy2 = point.getY() - this.point.getY();

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
      direction.negate();
   }

   public Line2d negateDirectionCopy()
   {
      Line2d ret = new Line2d(this);
      ret.direction.negate();

      return ret;
   }

   public void setPoint2d(Point2DReadOnly point2d)
   {
      point.set(point2d);
   }

   public void set(Point2DReadOnly endpoint0, Point2DReadOnly endpoint1)
   {
      if ((endpoint0.getX() == endpoint1.getX()) && (endpoint0.getY() == endpoint1.getY()))
      {
         throw new RuntimeException("Tried to set a line from two coincidal points.");
      }

      point.set(endpoint0);
      direction.set(endpoint1);
      direction.sub(endpoint0);
      direction.normalize();
   }

   public void set(double pointX, double pointY, double vectorX, double vectorY)
   {
      point.set(pointX, pointY);
      direction.set(vectorX, vectorY);
      checkReasonableVector(direction);
      direction.normalize();
   }

   public void set(Point2DReadOnly[] endpoints)
   {
      if (endpoints.length != 2)
         throw new RuntimeException("Length of input array is not correct. Length = " + endpoints.length);
      this.set(endpoints[0], endpoints[1]);
   }

   @Override
   public void set(Line2d line2d)
   {
      point.set(line2d.point);
      direction.set(line2d.direction);
   }

   public void rotate(double radians)
   {
      double vXOld = direction.getX();
      @SuppressWarnings("unused")
      double vYOld = direction.getY();

      double vXNew = Math.cos(radians) * vXOld - Math.sin(radians) * direction.getY();
      double vYNew = Math.sin(radians) * vXOld + Math.cos(radians) * direction.getY();

      direction.set(vXNew, vYNew);
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
      double vectorX = direction.getX();
      double vectorY = direction.getY();

      double vectorXPerpToRight = -vectorY;
      double vectorYPerpToRight = vectorX;

      if (!shiftToLeft)
      {
         vectorXPerpToRight = -vectorXPerpToRight;
         vectorYPerpToRight = -vectorYPerpToRight;
      }

      vectorXPerpToRight = distanceToShift * vectorXPerpToRight;
      vectorYPerpToRight = distanceToShift * vectorYPerpToRight;

      point.setX(point.getX() + vectorXPerpToRight);
      point.setY(point.getY() + vectorYPerpToRight);
   }

   public Line2d interiorBisector(Line2d secondLine)
   {
      Point2D pointOnLine = intersectionWith(secondLine);
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

      Vector2D directionVector = new Vector2D(direction);
      directionVector.add(secondLine.direction);

      return new Line2d(pointOnLine, directionVector);

   }

   public void perpendicularVector(Vector2DBasics vectorToPack)
   {
      vectorToPack.set(direction.getY(), -direction.getX());
   }

   public Vector2D perpendicularVector()
   {
      Vector2D vectorToReturn = new Vector2D();
      perpendicularVector(vectorToReturn);
      return vectorToReturn;
   }

   public Line2d perpendicularLineThroughPoint(Point2DReadOnly point)
   {
      return new Line2d(point, perpendicularVector());
   }

   public Point2D intersectionWith(LineSegment2d lineSegment)
   {
      return lineSegment.intersectionWith(this);
   }

   public boolean areLinesPerpendicular(Line2d line)
   {
      // Dot product of two vectors is zero if the vectors are perpendicular
      return direction.dot(line.getNormalizedVector()) < 1e-7;
   }

   public Point2D intersectionWith(Line2d secondLine)
   {
      return EuclidGeometryTools.intersectionBetweenTwoLine2Ds(point, direction, secondLine.point, secondLine.direction);
   }

   public boolean intersectionWith(Line2d secondLine, Point2DBasics intersectionToPack)
   {
      return EuclidGeometryTools.intersectionBetweenTwoLine2Ds(point, direction, secondLine.point, secondLine.direction, intersectionToPack);
   }

   public Point2D[] intersectionWith(ConvexPolygon2d convexPolygon)
   {
      return convexPolygon.intersectionWith(this);
   }

   public double distance(Point2DReadOnly point)
   {
      return EuclidGeometryTools.distanceFromPoint2DToLine2D(point, this.point, direction);
   }

   public double distanceSquared(Point2DReadOnly point)
   {
      double distance = distance(point);

      return distance * distance;
   }

   public double distance(Line2d line)
   {
      throw new RuntimeException("Not yet implemented");
   }

   public double distance(LineSegment2d lineSegment)
   {
      throw new RuntimeException("Not yet implemented");
   }

   public double distance(ConvexPolygon2d convexPolygon)
   {
      throw new RuntimeException("Not yet implemented");
   }

   @Override
   public String toString()
   {
      String ret = "";

      ret = ret + point + ", " + direction;

      return ret;
   }

   @Override
   public void applyTransform(Transform transform)
   {
      point.applyTransform(transform);
      direction.applyTransform(transform);
   }

   public void applyTransformAndProjectToXYPlane(Transform transform)
   {
      point.applyTransform(transform, false);
      direction.applyTransform(transform, false);
   }

   public Line2d applyTransformCopy(Transform transform)
   {
      Line2d copy = new Line2d(this);
      copy.applyTransform(transform);
      return copy;
   }

   public Line2d applyTransformAndProjectToXYPlaneCopy(Transform transform)
   {
      Line2d copy = new Line2d(this);
      copy.applyTransformAndProjectToXYPlane(transform);
      return copy;
   }

   public boolean isPointOnLine(Point2DReadOnly point)
   {
      double epsilon = 1e-8;
      if (Math.abs(direction.getX()) < 10E-10)
         return MathTools.epsilonEquals(point.getX(), this.point.getX(), epsilon);

      // y = A*x + b with point = (x,y)
      double A = direction.getY() / direction.getX();
      double b = this.point.getY() - A * this.point.getX();

      double value = point.getY() - A * point.getX() - b;

      return epsilon > Math.abs(value);
   }

   public boolean isPointOnLeftSideOfLine(Point2DReadOnly point)
   {
      return isPointOnSideOfLine(point.getX(), point.getY(), RobotSide.LEFT);
   }

   public boolean isPointOnRightSideOfLine(Point2DReadOnly point)
   {
      return isPointOnSideOfLine(point.getX(), point.getY(), RobotSide.RIGHT);
   }

   public boolean isPointOnSideOfLine(Point2DReadOnly point, RobotSide side)
   {
      return isPointOnSideOfLine(point.getX(), point.getY(), side);
   }

   private boolean isPointOnSideOfLine(double x, double y, RobotSide side)
   {
      return EuclidGeometryTools.isPoint2DOnSideOfLine2D(x, y, point, direction, side == RobotSide.LEFT);
   }

   /**
    * This method could be improved but must be tested better first.
    */
   public boolean isPointInFrontOfLine(Vector2DReadOnly frontDirection, Point2DReadOnly point)
   {
      double lineAngle = AngleTools.angleFromZeroToTwoPi(direction.getX(), direction.getY());
      double frontAngle = AngleTools.angleFromZeroToTwoPi(frontDirection.getX(), frontDirection.getY());
      double pointAngle = AngleTools.angleFromZeroToTwoPi(point.getX() - this.point.getX(), point.getY() - this.point.getY());

      double lineToFront = frontAngle - lineAngle;
      double lineToPoint = pointAngle - lineAngle;

      if (Math.abs(lineToFront) > Math.PI)
         lineToFront = -(lineToFront % Math.PI);
      if (Math.abs(lineToPoint) > Math.PI)
         lineToPoint = -(lineToPoint % Math.PI);

      if (lineToFront > 0.0 == lineToPoint > 0.0)
      {
         return true;
      }
      else
      {
         return false;
      }
   }

   /**
    * isPointInFrontOfLine returns whether the point is in front of the line or not. The front
    * direction is defined as the positive x-direction
    *
    * @param point Point2d
    * @return boolean
    */
   public boolean isPointInFrontOfLine(Point2DReadOnly point)
   {
      return isPointInFrontOfLine(point.getX(), point.getY());
   }

   private boolean isPointInFrontOfLine(double x, double y)
   {
      if (direction.getY() > 0.0)
         return isPointOnSideOfLine(x, y, RobotSide.RIGHT);
      else if (direction.getY() < 0.0)
         return isPointOnSideOfLine(x, y, RobotSide.LEFT);
      else
         throw new RuntimeException("Not defined when line is pointing exactly along the x-axis");
   }

   // TODO: Inconsistency in strictness.
   public boolean isPointBehindLine(Point2DReadOnly point)
   {
      return !isPointInFrontOfLine(point);
   }

   public void setParallelLineThroughPoint(Point2DReadOnly point)
   {
      this.point.set(point);
   }

   @Override
   public boolean containsNaN()
   {
      if (Double.isNaN(point.getX()))
         return true;
      if (Double.isNaN(point.getY()))
         return true;
      if (Double.isNaN(direction.getX()))
         return true;
      if (Double.isNaN(direction.getY()))
         return true;

      return false;
   }

   /**
    * Computes the orthogonal projection of the given 2D point on this 2D line.
    * 
    * @param point2d the point to project on this line. Modified.
    */
   public void orthogonalProjection(Point2DBasics point2d)
   {
      EuclidGeometryTools.orthogonalProjectionOnLine2D(point2d, point, direction, point2d);
   }

   /**
    * Computes the orthogonal projection of the given 2D point on this 2D line.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param point2d the point to compute the projection of. Not modified.
    * @return the projection of the point onto the line or {@code null} if the method failed.
    */
   public Point2D orthogonalProjectionCopy(Point2DReadOnly point2d)
   {
      Point2D projection = new Point2D();

      boolean success = EuclidGeometryTools.orthogonalProjectionOnLine2D(point2d, point, direction, projection);
      if (!success)
         return null;
      else
         return projection;
   }

   public boolean equals(Line2d otherLine)
   {
      return point.equals(otherLine.point) && direction.equals(otherLine.direction);
   }

   private void checkReasonableVector(Vector2DReadOnly localVector)
   {
      if ((Math.abs(localVector.getX()) < minAllowableVectorPart) && (Math.abs(localVector.getY()) < minAllowableVectorPart))
      {
         throw new RuntimeException("Line length must be greater than zero");
      }
   }

   private void checkDistinctPoints(Point2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine)
   {
      if ((firstPointOnLine.getX() == secondPointOnLine.getX()) && (firstPointOnLine.getY() == secondPointOnLine.getY()))
      {
         throw new RuntimeException("Tried to create a line from two coincidal points");
      }
   }

   @Override
   public void setToZero()
   {
      this.point.setToZero();
      this.direction.setToZero();
   }

   @Override
   public void setToNaN()
   {
      this.point.setToNaN();
      this.direction.setToNaN();
   }

   @Override
   public boolean epsilonEquals(Line2d other, double epsilon)
   {
      if (!this.point.epsilonEquals(other.point, epsilon))
         return false;
      if (!this.direction.epsilonEquals(other.direction, epsilon))
         return false;

      return true;
   }
}
