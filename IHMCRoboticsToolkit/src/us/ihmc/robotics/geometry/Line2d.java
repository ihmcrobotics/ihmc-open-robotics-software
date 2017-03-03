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
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * @author Twan Koolen
 */
public class Line2d implements GeometryObject<Line2d>
{
   // TODO: think about usage of epsilons in the methods.

   protected final Point2D point = new Point2D();
   protected final Vector2D normalizedVector = new Vector2D();
   private final static double doubleMinNormal; // Double.MIN_NORMAL is not available in JRE 1.5

   private final Point2D tempPoint2d = new Point2D();

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

   public Line2d(Point2DReadOnly point, Vector2DReadOnly vector)
   {
      this.point.set(point);
      normalizedVector.set(vector);
      checkReasonableVector(normalizedVector);
      normalizedVector.normalize();
   }

   public Line2d(Point2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine)
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

   public void set(Point2DReadOnly pointOnLine, Vector2DReadOnly vectorAlongLine)
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
      normalizedVectorToPack.set(normalizedVector);
   }

   public Vector2D getNormalizedVector()
   {
      return normalizedVector;
   }

   public void getPointAndNormalizedVector(Point2DBasics pointToPack, Vector2DBasics normalizedVectorToPack)
   {
      getPoint(pointToPack);
      getNormalizedVector(normalizedVectorToPack);
   }

   public void getTwoPointsOnLine(Point2DBasics point1, Point2DBasics point2)
   {
      point1.set(point);
      point2.add(point, normalizedVector);
   }

   public double getSlope()
   {
      if ((normalizedVector.getX() == 0.0) && (normalizedVector.getY() > 0.0))
      {
         return Double.POSITIVE_INFINITY;
      }

      if ((normalizedVector.getX() == 0.0) && (normalizedVector.getY() < 0.0))
      {
         return Double.NEGATIVE_INFINITY;
      }

      return normalizedVector.getY() / normalizedVector.getX();
   }

   public void getPointGivenParameter(double t, Point2DBasics pointToPack)
   {
      pointToPack.set(point);

      pointToPack.setX(pointToPack.getX() + t * normalizedVector.getX());
      pointToPack.setY(pointToPack.getY() + t * normalizedVector.getY());
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

         return Math.signum(difference.dot(normalizedVector)) * difference.length();
      }
   }

   public double getXIntercept()
   {
      double parameterAtIntercept = -point.getY() / normalizedVector.getY();

      return getPointGivenParameter(parameterAtIntercept).getX();
   }

   public double getYIntercept()
   {
      double parameterAtIntercept = -point.getX() / normalizedVector.getX();

      return getPointGivenParameter(parameterAtIntercept).getY();
   }

   public boolean containsEpsilon(Point2DReadOnly point, double epsilon)
   {
      // TODO: possibility to reduce code duplication by calling Geometry2dCalculator.distanceSquared
      // TODO: Refactor such that it is clear that this checks wether the point is within distance epsilon^2 from the line
      double vx1 = normalizedVector.getX();
      double vy1 = normalizedVector.getY();

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
      normalizedVector.negate();
   }

   public Line2d negateDirectionCopy()
   {
      Line2d ret = new Line2d(this);
      ret.normalizedVector.negate();

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
      normalizedVector.set(endpoint1);
      normalizedVector.sub(endpoint0);
      normalizedVector.normalize();
   }

   public void set(double pointX, double pointY, double vectorX, double vectorY)
   {
      if ((Math.abs(vectorX) < minAllowableVectorPart) && (Math.abs(vectorY) < minAllowableVectorPart))
      {
         throw new RuntimeException("Line length must be greater than zero");
      }

      point.set(pointX, pointY);
      normalizedVector.set(vectorX, vectorY);
      normalizedVector.normalize();
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
      normalizedVector.set(line2d.normalizedVector);
   }

   public void rotate(double radians)
   {
      double vXOld = normalizedVector.getX();
      @SuppressWarnings("unused")
      double vYOld = normalizedVector.getY();

      double vXNew = Math.cos(radians) * vXOld - Math.sin(radians) * normalizedVector.getY();
      double vYNew = Math.sin(radians) * vXOld + Math.cos(radians) * normalizedVector.getY();

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
      double vectorX = normalizedVector.getX();
      double vectorY = normalizedVector.getY();

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

      Vector2D directionVector = new Vector2D(normalizedVector);
      directionVector.add(secondLine.normalizedVector);

      return new Line2d(pointOnLine, directionVector);

   }

   public void perpendicularVector(Vector2DBasics vectorToPack)
   {
      vectorToPack.set(normalizedVector.getY(), -normalizedVector.getX());
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
      return normalizedVector.dot(line.getNormalizedVector()) < 1e-7;
   }

   public Point2D intersectionWith(Line2d secondLine)
   {
      return GeometryTools.getIntersectionBetweenTwoLines(point, normalizedVector, secondLine.point, secondLine.normalizedVector);
   }

   public boolean intersectionWith(Line2d secondLine, Point3DBasics intersectionToPack)
   {
      boolean success = GeometryTools.getIntersectionBetweenTwoLines(point, normalizedVector, secondLine.point, secondLine.normalizedVector, tempPoint2d);
      if (success)
         intersectionToPack.set(tempPoint2d.getX(), tempPoint2d.getY(), intersectionToPack.getZ());
      return success;
   }

   public boolean intersectionWith(Line2d secondLine, Point2DBasics intersectionToPack)
   {
      return GeometryTools.getIntersectionBetweenTwoLines(point, normalizedVector, secondLine.point, secondLine.normalizedVector, intersectionToPack);
   }

   public Point2D[] intersectionWith(ConvexPolygon2d convexPolygon)
   {
      return convexPolygon.intersectionWith(this);
   }

   public double distance(Point2DReadOnly point)
   {
      tempPoint2d.set(this.point);
      tempPoint2d.add(normalizedVector);

      return EuclidGeometryTools.distanceFromPoint2DToLine2D(point, this.point, tempPoint2d);
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

      ret = ret + point + ", " + normalizedVector;

      return ret;
   }

   @Override
   public void applyTransform(Transform transform)
   {
      point.applyTransform(transform);
      normalizedVector.applyTransform(transform);
   }

   public void applyTransformAndProjectToXYPlane(Transform transform)
   {
      point.applyTransform(transform, false);
      normalizedVector.applyTransform(transform, false);
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
      if (Math.abs(normalizedVector.getX()) < 10E-10)
         return MathTools.epsilonEquals(point.getX(), this.point.getX(), epsilon);

      // y = A*x + b with point = (x,y)
      double A = normalizedVector.getY() / normalizedVector.getX();
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
      return EuclidGeometryTools.isPoint2DOnSideOfLine2D(x, y, point, normalizedVector, side == RobotSide.LEFT);
   }

   /**
    * This method could be improved but must be tested better first.
    */
   public boolean isPointInFrontOfLine(Vector2DReadOnly frontDirection, Point2DReadOnly point)
   {
      double lineAngle = AngleTools.angleFromZeroToTwoPi(normalizedVector.getX(), normalizedVector.getY());
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
      if (normalizedVector.getY() > 0.0)
         return isPointOnSideOfLine(x, y, RobotSide.RIGHT);
      else if (normalizedVector.getY() < 0.0)
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
      if (Double.isNaN(normalizedVector.getX()))
         return true;
      if (Double.isNaN(normalizedVector.getY()))
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
      GeometryTools.getOrthogonalProjectionOnLine(point2d, point, normalizedVector, point2d);
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

      boolean success = GeometryTools.getOrthogonalProjectionOnLine(point2d, point, normalizedVector, projection);
      if (!success)
         return null;
      else
         return projection;
   }

   public boolean equals(Line2d otherLine)
   {
      return point.equals(otherLine.point) && normalizedVector.equals(otherLine.normalizedVector);
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
      this.normalizedVector.setToZero();
   }

   @Override
   public void setToNaN()
   {
      this.point.setToNaN();
      this.normalizedVector.setToNaN();
   }

   @Override
   public boolean epsilonEquals(Line2d other, double epsilon)
   {
      if (!this.point.epsilonEquals(other.point, epsilon))
         return false;
      if (!this.normalizedVector.epsilonEquals(other.normalizedVector, epsilon))
         return false;

      return true;
   }
}
