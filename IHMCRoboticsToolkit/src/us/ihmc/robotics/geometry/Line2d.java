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
   private final Point2D point = new Point2D();
   /** Normalized direction of this line. */
   private final Vector2D direction = new Vector2D();

   private boolean hasPointBeenSet = false;
   private boolean hasDirectionBeenSet = false;

   public Line2d()
   {
      hasPointBeenSet = false;
      hasDirectionBeenSet = false;
   }

   public Line2d(Line2d other)
   {
      set(other);
   }

   public Line2d(Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection)
   {
      set(pointOnLine, lineDirection);
   }

   public Line2d(Point2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine)
   {
      set(firstPointOnLine, secondPointOnLine);
   }

   public Line2d(double pointOnLineX, double pointOnLineY, double lineDirectionX, double lineDirectionY)
   {
      set(pointOnLineX, pointOnLineY, lineDirectionX, lineDirectionY);
   }

   @Override
   public void setToZero()
   {
      point.setToZero();
      direction.setToZero();
   }

   @Override
   public void setToNaN()
   {
      point.setToNaN();
      direction.setToNaN();
   }

   @Override
   public boolean containsNaN()
   {
      return point.containsNaN() || direction.containsNaN();
   }

   public void setPoint(double pointOnLineX, double pointOnLineY)
   {
      point.set(pointOnLineX, pointOnLineY);
      hasPointBeenSet = true;
   }

   public void setPoint(Point2DReadOnly pointOnLine)
   {
      setPoint(pointOnLine.getX(), pointOnLine.getY());
   }

   public void setDirection(double lineDirectionX, double lineDirectionY)
   {
      direction.set(lineDirectionX, lineDirectionY);
      checkReasonableVector(direction);
      direction.normalize();
      hasDirectionBeenSet = true;
   }

   public void setDirection(Vector2DReadOnly lineDirection)
   {
      setDirection(lineDirection.getX(), lineDirection.getY());
   }

   @Override
   public void set(Line2d other)
   {
      set(other.point, other.direction);
   }

   public void set(Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection)
   {
      setPoint(pointOnLine);
      setDirection(lineDirection);
   }

   public void set(Point2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine)
   {
      checkDistinctPoints(firstPointOnLine, secondPointOnLine);
      setPoint(firstPointOnLine);
      setDirection(secondPointOnLine.getX() - firstPointOnLine.getX(), secondPointOnLine.getY() - firstPointOnLine.getY());
   }

   public void set(double pointOnLineX, double pointOnLineY, double lineDirectionX, double lineDirectionY)
   {
      setPoint(pointOnLineX, pointOnLineY);
      setDirection(lineDirectionX, lineDirectionY);
   }

   public void set(Point2DReadOnly[] endpoints)
   {
      if (endpoints.length != 2)
         throw new IllegalArgumentException("Length of input array is not correct. Length = " + endpoints.length + ", expected an array of two elements");
      set(endpoints[0], endpoints[1]);
   }

   public void getPoint(Point2DBasics pointOnLineToPack)
   {
      pointOnLineToPack.set(point);
   }

   public Point2DReadOnly getPoint()
   {
      return point;
   }

   public double getPointX()
   {
      return point.getX();
   }

   public double getPointY()
   {
      return point.getY();
   }

   public void getDirection(Vector2DBasics directionToPack)
   {
      checkHasBeenInitialized();
      directionToPack.set(direction);
   }

   public Vector2DReadOnly getDirection()
   {
      checkHasBeenInitialized();
      return direction;
   }

   public double getDirectionX()
   {
      checkHasBeenInitialized();
      return direction.getX();
   }

   public double getDirectionY()
   {
      checkHasBeenInitialized();
      return direction.getY();
   }

   public void getPointAndDirection(Point2DBasics pointToPack, Vector2DBasics directionToPack)
   {
      getPoint(pointToPack);
      getDirection(directionToPack);
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
      checkHasBeenInitialized();
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
      checkHasBeenInitialized();

      double parameterAtIntercept = -point.getY() / direction.getY();

      return getPointGivenParameter(parameterAtIntercept).getX();
   }

   public double getYIntercept()
   {
      checkHasBeenInitialized();
      double parameterAtIntercept = -point.getX() / direction.getX();

      return getPointGivenParameter(parameterAtIntercept).getY();
   }

   public boolean containsEpsilon(Point2DReadOnly point, double epsilon)
   {
      checkHasBeenInitialized();
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
      checkHasBeenInitialized();
      direction.negate();
   }

   public Line2d negateDirectionCopy()
   {
      checkHasBeenInitialized();
      Line2d ret = new Line2d(this);
      ret.direction.negate();

      return ret;
   }

   public void rotate(double radians)
   {
      checkHasBeenInitialized();
      double vXOld = direction.getX();
      double vYOld = direction.getY();

      double vXNew = Math.cos(radians) * vXOld - Math.sin(radians) * vYOld;
      double vYNew = Math.sin(radians) * vXOld + Math.cos(radians) * vYOld;

      direction.set(vXNew, vYNew);

   }

   public void translate(double x, double y)
   {
      point.add(x, y);
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
      checkHasBeenInitialized();
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
      checkHasBeenInitialized();
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
      checkHasBeenInitialized();
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
      checkHasBeenInitialized();
      return new Line2d(point, perpendicularVector());
   }

   public Point2D intersectionWith(LineSegment2d lineSegment)
   {
      checkHasBeenInitialized();
      return lineSegment.intersectionWith(this);
   }

   public boolean areLinesPerpendicular(Line2d line)
   {
      checkHasBeenInitialized();
      // Dot product of two vectors is zero if the vectors are perpendicular
      return direction.dot(line.getDirection()) < 1e-7;
   }

   public Point2D intersectionWith(Line2d secondLine)
   {
      checkHasBeenInitialized();
      return EuclidGeometryTools.intersectionBetweenTwoLine2Ds(point, direction, secondLine.point, secondLine.direction);
   }

   public boolean intersectionWith(Line2d secondLine, Point2DBasics intersectionToPack)
   {
      checkHasBeenInitialized();
      return EuclidGeometryTools.intersectionBetweenTwoLine2Ds(point, direction, secondLine.point, secondLine.direction, intersectionToPack);
   }

   public Point2D[] intersectionWith(ConvexPolygon2d convexPolygon)
   {
      checkHasBeenInitialized();
      return convexPolygon.intersectionWith(this);
   }

   public double distance(Point2DReadOnly point)
   {
      checkHasBeenInitialized();
      return EuclidGeometryTools.distanceFromPoint2DToLine2D(point, this.point, direction);
   }

   public double distanceSquared(Point2DReadOnly point)
   {
      checkHasBeenInitialized();
      double distance = distance(point);

      return distance * distance;
   }

   public double distance(Line2d line)
   {
      checkHasBeenInitialized();
      throw new RuntimeException("Not yet implemented");
   }

   public double distance(LineSegment2d lineSegment)
   {
      checkHasBeenInitialized();
      throw new RuntimeException("Not yet implemented");
   }

   public double distance(ConvexPolygon2d convexPolygon)
   {
      checkHasBeenInitialized();
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
      checkHasBeenInitialized();
      point.applyTransform(transform);
      direction.applyTransform(transform);
   }

   public void applyTransformAndProjectToXYPlane(Transform transform)
   {
      checkHasBeenInitialized();
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
      checkHasBeenInitialized();
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
      checkHasBeenInitialized();
      return EuclidGeometryTools.isPoint2DOnSideOfLine2D(x, y, point, direction, side == RobotSide.LEFT);
   }

   /**
    * This method could be improved but must be tested better first.
    */
   public boolean isPointInFrontOfLine(Vector2DReadOnly frontDirection, Point2DReadOnly point)
   {
      double crossProduct = frontDirection.cross(direction);
      if (crossProduct > 0.0)
         return isPointOnRightSideOfLine(point);
      else if (crossProduct < 0.0)
         return isPointOnLeftSideOfLine(point);
      else
         throw new RuntimeException("Not defined when line is pointing exactly along the front direction");
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

   /**
    * Computes the orthogonal projection of the given 2D point on this 2D line.
    *
    * @param point2d the point to project on this line. Modified.
    */
   public void orthogonalProjection(Point2DBasics point2d)
   {
      checkHasBeenInitialized();
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
      checkHasBeenInitialized();
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

   private void checkHasBeenInitialized()
   {
      if (!hasPointBeenSet)
         throw new RuntimeException("The point of this line has not been initialized.");
      if (!hasDirectionBeenSet)
         throw new RuntimeException("The direction of this line has not been initialized.");
   }

   @Override
   public boolean epsilonEquals(Line2d other, double epsilon)
   {
      checkHasBeenInitialized();
      if (!point.epsilonEquals(other.point, epsilon))
         return false;
      if (!direction.epsilonEquals(other.direction, epsilon))
         return false;

      return true;
   }
}
