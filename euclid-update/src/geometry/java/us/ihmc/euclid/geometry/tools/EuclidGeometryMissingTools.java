package us.ihmc.euclid.geometry.tools;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.CommonMatrix3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;

import java.util.List;

import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.ONE_MILLIONTH;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.ONE_TRILLIONTH;
import static us.ihmc.euclid.tools.EuclidCoreTools.normSquared;

public class EuclidGeometryMissingTools
{
   /**
    * Tests if the point 2D is located on the infinitely long line 2D.
    * <p>
    * The test is performed by computing the distance between the point and the line, if that distance
    * is below {@link EuclidGeometryTools#IS_POINT_ON_LINE_EPS} this method returns {@code true}.
    * </p>
    *
    * @param lineSegmentStart the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   the second endpoint of the line segment. Not modified.
    * @return {@code true} if the query is considered to be lying on the line, {@code false} otherwise.
    */
   public static boolean isPoint2DOnLineSegment2D(double pointX, double pointY, Point2DReadOnly lineSegmentStart, Point2DReadOnly lineSegmentEnd)
   {
      return EuclidGeometryTools.distanceSquaredFromPoint2DToLineSegment2D(pointX,
                                                                           pointY,
                                                                           lineSegmentStart.getX(),
                                                                           lineSegmentStart.getY(),
                                                                           lineSegmentEnd.getX(),
                                                                           lineSegmentEnd.getY()) < EuclidGeometryTools.IS_POINT_ON_LINE_EPS;
   }

   public static boolean intersectionBetweenTwoLine2Ds(Point2DReadOnly firstPointOnLine1,
                                                       Point2DReadOnly secondPointOnLine1,
                                                       Point2DReadOnly firstPointOnLine2,
                                                       Point2DReadOnly secondPointOnLine2,
                                                       Point2DBasics intersectionToPack)
   {
      double pointOnLine1x = firstPointOnLine1.getX();
      double pointOnLine1y = firstPointOnLine1.getY();
      double lineDirection1x = secondPointOnLine1.getX() - firstPointOnLine1.getX();
      double lineDirection1y = secondPointOnLine1.getY() - firstPointOnLine1.getY();
      double pointOnLine2x = firstPointOnLine2.getX();
      double pointOnLine2y = firstPointOnLine2.getY();
      double lineDirection2x = secondPointOnLine2.getX() - firstPointOnLine2.getX();
      double lineDirection2y = secondPointOnLine2.getY() - firstPointOnLine2.getY();
      return EuclidGeometryTools.intersectionBetweenTwoLine2Ds(pointOnLine1x,
                                                               pointOnLine1y,
                                                               lineDirection1x,
                                                               lineDirection1y,
                                                               pointOnLine2x,
                                                               pointOnLine2y,
                                                               lineDirection2x,
                                                               lineDirection2y,
                                                               intersectionToPack);
   }

   /**
    * This method implements the same operation as
    * {@link EuclidGeometryTools#orientation3DFromFirstToSecondVector3D(double, double, double, double, double, double, Orientation3DBasics)}
    * except that it does not rely on {@code Math#acos(double)} making it faster.
    *
    * @param firstVectorX   x-component of the first vector.
    * @param firstVectorY   y-component of the first vector.
    * @param firstVectorZ   z-component of the first vector.
    * @param secondVectorX  x-component of the second vector that is rotated with respect to the first
    *                       vector.
    * @param secondVectorY  y-component of the second vector that is rotated with respect to the first
    *                       vector.
    * @param secondVectorZ  z-component of the second vector that is rotated with respect to the first
    *                       vector.
    * @param rotationToPack the minimum rotation from {@code firstVector} to the {@code secondVector}.
    *                       Modified.
    * @see EuclidGeometryTools#orientation3DFromFirstToSecondVector3D(double, double, double, double,
    *       double, double, Orientation3DBasics)
    */
   public static void rotationMatrix3DFromFirstToSecondVector3D(double firstVectorX,
                                                                double firstVectorY,
                                                                double firstVectorZ,
                                                                double secondVectorX,
                                                                double secondVectorY,
                                                                double secondVectorZ,
                                                                CommonMatrix3DBasics rotationToPack)
   {
      double firstVectorLengthInv = 1.0 / Math.sqrt(normSquared(firstVectorX, firstVectorY, firstVectorZ));
      double secondVectorLengthInv = 1.0 / Math.sqrt(normSquared(secondVectorX, secondVectorY, secondVectorZ));
      firstVectorX *= firstVectorLengthInv;
      firstVectorY *= firstVectorLengthInv;
      firstVectorZ *= firstVectorLengthInv;
      secondVectorX *= secondVectorLengthInv;
      secondVectorY *= secondVectorLengthInv;
      secondVectorZ *= secondVectorLengthInv;

      double rotationAxisX = firstVectorY * secondVectorZ - firstVectorZ * secondVectorY;
      double rotationAxisY = firstVectorZ * secondVectorX - firstVectorX * secondVectorZ;
      double rotationAxisZ = firstVectorX * secondVectorY - firstVectorY * secondVectorX;
      double sinAngle = Math.sqrt(normSquared(rotationAxisX, rotationAxisY, rotationAxisZ));

      boolean normalsAreParallel = sinAngle < EuclidGeometryTools.ONE_TEN_MILLIONTH;

      if (normalsAreParallel)
      {
         rotationToPack.setIdentity();
         return;
      }

      rotationAxisX /= sinAngle;
      rotationAxisY /= sinAngle;
      rotationAxisZ /= sinAngle;

      double cosAngle = firstVectorX * secondVectorX + firstVectorY * secondVectorY + firstVectorZ * secondVectorZ;

      if (cosAngle > 1.0)
         cosAngle = 1.0;
      else if (cosAngle < -1.0)
         cosAngle = -1.0;

      double t = 1.0 - cosAngle;

      double xz = rotationAxisX * rotationAxisZ;
      double xy = rotationAxisX * rotationAxisY;
      double yz = rotationAxisY * rotationAxisZ;

      double m00 = t * rotationAxisX * rotationAxisX + cosAngle;
      double m01 = t * xy - sinAngle * rotationAxisZ;
      double m02 = t * xz + sinAngle * rotationAxisY;
      double m10 = t * xy + sinAngle * rotationAxisZ;
      double m11 = t * rotationAxisY * rotationAxisY + cosAngle;
      double m12 = t * yz - sinAngle * rotationAxisX;
      double m20 = t * xz - sinAngle * rotationAxisY;
      double m21 = t * yz + sinAngle * rotationAxisX;
      double m22 = t * rotationAxisZ * rotationAxisZ + cosAngle;

      if (rotationToPack instanceof RotationMatrix)
         ((RotationMatrix) rotationToPack).setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      else
         rotationToPack.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * This method implements the same operation as
    * {@link EuclidGeometryTools#orientation3DFromFirstToSecondVector3D(Vector3DReadOnly, Vector3DReadOnly, Orientation3DBasics)}
    * except that it does not rely on {@code Math#acos(double)} making it faster.
    *
    * @param firstVector    the first vector. Not modified.
    * @param secondVector   the second vector that is rotated with respect to the first vector. Not
    *                       modified.
    * @param rotationToPack the minimum rotation from {@code firstVector} to the {@code secondVector}.
    *                       Modified.
    * @see EuclidGeometryTools#orientation3DFromFirstToSecondVector3D(Vector3DReadOnly,
    *       Vector3DReadOnly, Orientation3DBasics)
    */
   public static void rotationMatrix3DFromFirstToSecondVector3D(Vector3DReadOnly firstVector,
                                                                Vector3DReadOnly secondVector,
                                                                CommonMatrix3DBasics rotationToPack)
   {
      rotationMatrix3DFromFirstToSecondVector3D(firstVector.getX(),
                                                firstVector.getY(),
                                                firstVector.getZ(),
                                                secondVector.getX(),
                                                secondVector.getY(),
                                                secondVector.getZ(),
                                                rotationToPack);
   }

   /**
    * This method implements the same operation as
    * {@link EuclidGeometryTools#orientation3DFromZUpToVector3D(Vector3DReadOnly, Orientation3DBasics)}
    * except that it does not rely on {@code Math#acos(double)} making it faster.
    *
    * @param vector         the vector that is rotated with respect to {@code zUp}. Not modified.
    * @param rotationToPack the minimum rotation from {@code zUp} to the given {@code vector}.
    *                       Modified.
    * @see EuclidGeometryTools#orientation3DFromZUpToVector3D(Vector3DReadOnly, Orientation3DBasics)
    */
   public static void rotationMatrix3DFromZUpToVector3D(Vector3DReadOnly vector, CommonMatrix3DBasics rotationToPack)
   {
      rotationMatrix3DFromFirstToSecondVector3D(Axis3D.Z, vector, rotationToPack);
   }

   /**
    * Given two 2D line segments with finite length, this methods computes two points P &in;
    * lineSegment1 and Q &in; lineSegment2 such that the distance || P - Q || is the minimum distance
    * between the two 2D line segments. <a href="http://geomalgorithms.com/a07-_distance.html"> Useful
    * link</a>.
    *
    * @param lineSegmentStart1                the first endpoint of the first line segment. Not
    *                                         modified.
    * @param lineSegmentEnd1                  the second endpoint of the first line segment. Not
    *                                         modified.
    * @param lineSegmentStart2                the first endpoint of the second line segment. Not
    *                                         modified.
    * @param lineSegmentEnd2                  the second endpoint of the second line segment. Not
    *                                         modified.
    * @param closestPointOnLineSegment1ToPack the 2D coordinates of the point P are packed in this 2D
    *                                         point. Modified. Can be {@code null}.
    * @param closestPointOnLineSegment2ToPack the 2D coordinates of the point Q are packed in this 2D
    *                                         point. Modified. Can be {@code null}.
    * @return the minimum distance between the two line segments.
    */
   public static double closestPoint2DsBetweenTwoLineSegment2Ds(Point2DReadOnly lineSegmentStart1,
                                                                Point2DReadOnly lineSegmentEnd1,
                                                                Point2DReadOnly lineSegmentStart2,
                                                                Point2DReadOnly lineSegmentEnd2,
                                                                Point2DBasics closestPointOnLineSegment1ToPack,
                                                                Point2DBasics closestPointOnLineSegment2ToPack)
   {
      // Switching to the notation used in http://geomalgorithms.com/a07-_distance.html.
      // The line1 is defined by (P0, u) and the line2 by (Q0, v).
      Point2DReadOnly P0 = lineSegmentStart1;
      double ux = lineSegmentEnd1.getX() - lineSegmentStart1.getX();
      double uy = lineSegmentEnd1.getY() - lineSegmentStart1.getY();
      Point2DReadOnly Q0 = lineSegmentStart2;
      double vx = lineSegmentEnd2.getX() - lineSegmentStart2.getX();
      double vy = lineSegmentEnd2.getY() - lineSegmentStart2.getY();

      Point2DBasics Psc = closestPointOnLineSegment1ToPack;
      Point2DBasics Qtc = closestPointOnLineSegment2ToPack;

      double w0X = P0.getX() - Q0.getX();
      double w0Y = P0.getY() - Q0.getY();

      double a = ux * ux + uy * uy;
      double b = ux * vx + uy * vy;
      double c = vx * vx + vy * vy;
      double d = ux * w0X + uy * w0Y;
      double e = vx * w0X + vy * w0Y;

      double delta = a * c - b * b;

      double sc, sNumerator, sDenominator = delta;
      double tc, tNumerator, tDenominator = delta;

      // check to see if the lines are parallel
      if (delta <= ONE_MILLIONTH)
      {
         /*
          * The lines are parallel, there's an infinite number of pairs, but for one chosen point on one of
          * the lines, there's only one closest point to it on the other line. So let's choose arbitrarily a
          * point on the lineSegment1 and calculate the point that is closest to it on the lineSegment2.
          */
         sNumerator = 0.0;
         sDenominator = 1.0;
         tNumerator = e;
         tDenominator = c;
      }
      else
      {
         sNumerator = b * e - c * d;
         tNumerator = a * e - b * d;

         if (sNumerator < 0.0)
         {
            sNumerator = 0.0;
            tNumerator = e;
            tDenominator = c;
         }
         else if (sNumerator > sDenominator)
         {
            sNumerator = sDenominator;
            tNumerator = e + b;
            tDenominator = c;
         }
      }

      if (tNumerator < 0.0)
      {
         tNumerator = 0.0;
         sNumerator = -d;
         if (sNumerator < 0.0)
            sNumerator = 0.0;
         else if (sNumerator > a)
            sNumerator = a;
         sDenominator = a;
      }
      else if (tNumerator > tDenominator)
      {
         tNumerator = tDenominator;
         sNumerator = -d + b;
         if (sNumerator < 0.0)
            sNumerator = 0.0;
         else if (sNumerator > a)
            sNumerator = a;
         sDenominator = a;
      }

      sc = Math.abs(sNumerator) < ONE_MILLIONTH ? 0.0 : sNumerator / sDenominator;
      tc = Math.abs(tNumerator) < ONE_MILLIONTH ? 0.0 : tNumerator / tDenominator;

      double PscX = sc * ux + P0.getX();
      double PscY = sc * uy + P0.getY();

      double QtcX = tc * vx + Q0.getX();
      double QtcY = tc * vy + Q0.getY();

      if (Psc != null)
         Psc.set(PscX, PscY);
      if (Qtc != null)
         Qtc.set(QtcX, QtcY);

      double dx = PscX - QtcX;
      double dy = PscY - QtcY;
      return Math.sqrt(normSquared(dx, dy));
   }

   /**
    * This methods computes the minimum distance between the two 2D line segments with finite length.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param lineSegmentStart1 the first endpoint of the first line segment. Not modified.
    * @param lineSegmentEnd1   the second endpoint of the first line segment. Not modified.
    * @param lineSegmentStart2 the first endpoint of the second line segment. Not modified.
    * @param lineSegmentEnd2   the second endpoint of the second line segment. Not modified.
    * @return the minimum distance between the two line segments.
    */
   public static double distanceBetweenTwoLineSegment2Ds(Point2DReadOnly lineSegmentStart1,
                                                         Point2DReadOnly lineSegmentEnd1,
                                                         Point2DReadOnly lineSegmentStart2,
                                                         Point2DReadOnly lineSegmentEnd2)
   {
      return closestPoint2DsBetweenTwoLineSegment2Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2, null, null);
   }

   /**
    * Returns the square of the minimum distance between a point and a given line segment, and packs the closest point on the line segment in
    * {@code pointOnSegmentToPack}
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@value #ONE_TRILLIONTH}, this
    * method returns the distance between {@code lineSegmentStart} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param pointX               x coordinate of point to be tested.
    * @param pointY               y coordinate of point to be tested.
    * @param lineSegmentStartX    the x-coordinate of the line segment first endpoint.
    * @param lineSegmentStartY    the y-coordinate of the line segment first endpoint.
    * @param lineSegmentEndX      the x-coordinate of the line segment second endpoint.
    * @param lineSegmentEndY      the y-coordinate of the line segment second endpoint.
    * @param pointOnSegmentToPack the closest point on the line segment to the point to be tested. Modified.
    * @return the square of the minimum distance between the 2D point and the 2D line segment.
    */
   public static double distanceSquaredFromPoint2DToLineSegment2D(double pointX,
                                                                  double pointY,
                                                                  double lineSegmentStartX,
                                                                  double lineSegmentStartY,
                                                                  double lineSegmentEndX,
                                                                  double lineSegmentEndY,
                                                                  Point2DBasics pointOnSegmentToPack)
   {
      double percentage = EuclidGeometryTools.percentageAlongLineSegment2D(pointX,
                                                                           pointY,
                                                                           lineSegmentStartX,
                                                                           lineSegmentStartY,
                                                                           lineSegmentEndX,
                                                                           lineSegmentEndY);

      if (percentage > 1.0)
         percentage = 1.0;
      else if (percentage < 0.0)
         percentage = 0.0;

      double projectionX = (1.0 - percentage) * lineSegmentStartX + percentage * lineSegmentEndX;
      double projectionY = (1.0 - percentage) * lineSegmentStartY + percentage * lineSegmentEndY;

      if (pointOnSegmentToPack != null)
         pointOnSegmentToPack.set(projectionX, projectionY);

      double dx = projectionX - pointX;
      double dy = projectionY - pointY;
      return dx * dx + dy * dy;
   }

   public static int intersectionBetweenLineSegment2DAndCylinder3D(double circleRadius,
                                                                   Point2DReadOnly circlePosition,
                                                                   Point2DReadOnly startPoint,
                                                                   Point2DReadOnly endPoint,
                                                                   Point2DBasics firstIntersectionToPack,
                                                                   Point2DBasics secondIntersectionToPack)
   {
      return intersectionBetweenLine2DAndCircle(circleRadius,
                                                circlePosition.getX(),
                                                circlePosition.getY(),
                                                startPoint.getX(),
                                                startPoint.getY(),
                                                false,
                                                endPoint.getX(),
                                                endPoint.getY(),
                                                false,
                                                firstIntersectionToPack,
                                                secondIntersectionToPack);
   }

   public static int intersectionBetweenLine2DAndCircle(double circleRadius,
                                                        double circlePositionX,
                                                        double circlePositionY,
                                                        double startX,
                                                        double startY,
                                                        boolean canIntersectionOccurBeforeStart,
                                                        double endX,
                                                        double endY,
                                                        boolean canIntersectionOccurAfterEnd,
                                                        Point2DBasics firstIntersectionToPack,
                                                        Point2DBasics secondIntersectionToPack)
   {
      if (circleRadius < 0.0)
         throw new IllegalArgumentException("The circle radius has to be positive.");

      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setToNaN();
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setToNaN();

      if (circleRadius == 0.0)
         return 0;

      double radiusSquared = circleRadius * circleRadius;

      double dx = endX - startX;
      double dy = endY - startY;

      double dIntersection1 = Double.NaN;
      double dIntersection2 = Double.NaN;

      // Compute possible intersections with the circle
      //
      double deltaPX = startX - circlePositionX;
      double deltaPY = startY - circlePositionY;

      double A = EuclidCoreTools.normSquared(dx, dy);
      double B = 2.0 * (dx * deltaPX + dy * deltaPY);
      double C = EuclidCoreTools.normSquared(deltaPX, deltaPY) - radiusSquared;

      double delta = EuclidCoreTools.squareRoot(B * B - 4 * A * C);

      if (Double.isFinite(delta))
      {
         double oneOverTwoA = 0.5 / A;
         double dCircle1 = -(B + delta) * oneOverTwoA;
         double dCircle2 = -(B - delta) * oneOverTwoA;

         double intersection1X = dCircle1 * dx + startX;
         double intersection1Y = dCircle1 * dy + startY;

         if (Math.abs(EuclidGeometryTools.percentageAlongLine2D(intersection1X, intersection1Y, circlePositionX, circlePositionY, 1.0, 0.0))
             > circleRadius - ONE_TRILLIONTH)
            dCircle1 = Double.NaN;

         if (Double.isFinite(dCircle1))
         {
            if (Double.isNaN(dIntersection1) || Math.abs(dCircle1 - dIntersection1) < ONE_TRILLIONTH)
            {
               dIntersection1 = dCircle1;
            }
            else if (dCircle1 < dIntersection1)
            {
               dIntersection2 = dIntersection1;
               dIntersection1 = dCircle1;
            }
            else
            {
               dIntersection2 = dCircle1;
            }
         }

         double intersection2X = dCircle2 * dx + startX;
         double intersection2Y = dCircle2 * dy + startY;

         if (Math.abs(EuclidGeometryTools.percentageAlongLine2D(intersection2X, intersection2Y, circlePositionX, circlePositionY, 1.0, 0.0))
             > circleRadius - ONE_TRILLIONTH)
            dCircle2 = Double.NaN;
         else if (Math.abs(dCircle1 - dCircle2) < ONE_TRILLIONTH)
            dCircle2 = Double.NaN;

         if (Double.isFinite(dCircle2))
         {
            if (Double.isNaN(dIntersection1))
            {
               dIntersection1 = dCircle2;
            }
            else if (dCircle2 < dIntersection1)
            {
               dIntersection2 = dIntersection1;
               dIntersection1 = dCircle2;
            }
            else
            {
               dIntersection2 = dCircle2;
            }
         }
      }

      if (!canIntersectionOccurBeforeStart)
      {
         if (dIntersection2 < 0.0)
            dIntersection2 = Double.NaN;

         if (dIntersection1 < 0.0)
         {
            dIntersection1 = dIntersection2;
            dIntersection2 = Double.NaN;
         }
      }

      if (!canIntersectionOccurAfterEnd)
      {
         if (dIntersection2 > 1.0)
            dIntersection2 = Double.NaN;

         if (dIntersection1 > 1.0)
         {
            dIntersection1 = dIntersection2;
            dIntersection2 = Double.NaN;
         }
      }

      if (Double.isNaN(dIntersection1))
         return 0;

      if (firstIntersectionToPack != null)
      {
         firstIntersectionToPack.set(dx, dy);
         firstIntersectionToPack.scale(dIntersection1);
         firstIntersectionToPack.add(startX, startY);
      }

      if (Double.isNaN(dIntersection2))
         return 1;

      if (secondIntersectionToPack != null)
      {
         secondIntersectionToPack.set(dx, dy);
         secondIntersectionToPack.scale(dIntersection2);
         secondIntersectionToPack.add(startX, startY);
      }

      return 2;
   }

   public static int intersectionBetweenLine2DAndCircle(double circleRadius,
                                                        Point2DReadOnly circlePosition,
                                                        Point2DReadOnly pointOnLine,
                                                        Vector2DReadOnly direction,
                                                        Point2DBasics firstIntersectionToPack,
                                                        Point2DBasics secondIntersectionToPack)
   {
      return intersectionBetweenLine2DAndCircle(circleRadius,
                                                circlePosition.getX(),
                                                circlePosition.getY(),
                                                pointOnLine.getX(),
                                                pointOnLine.getY(),
                                                true,
                                                pointOnLine.getX() + direction.getX(),
                                                pointOnLine.getY() + direction.getY(),
                                                true,
                                                firstIntersectionToPack,
                                                secondIntersectionToPack);
   }

   public static int intersectionBetweenRay2DAndCircle(double circleRadius,
                                                       Point2DReadOnly circlePosition,
                                                       Point2DReadOnly startPoint,
                                                       Vector2DReadOnly direction,
                                                       Point2DBasics firstIntersectionToPack,
                                                       Point2DBasics secondIntersectionToPack)
   {
      return intersectionBetweenLine2DAndCircle(circleRadius,
                                                circlePosition.getX(),
                                                circlePosition.getY(),
                                                startPoint.getX(),
                                                startPoint.getY(),
                                                false,
                                                startPoint.getX() + direction.getX(),
                                                startPoint.getY() + direction.getY(),
                                                true,
                                                firstIntersectionToPack,
                                                secondIntersectionToPack);
   }

   public static int intersectionBetweenRay2DAndCircle(double circleRadius,
                                                       Point2DReadOnly circlePosition,
                                                       Point2DReadOnly startPoint,
                                                       Point2DReadOnly pointOnRay,
                                                       Point2DBasics firstIntersectionToPack,
                                                       Point2DBasics secondIntersectionToPack)
   {
      return intersectionBetweenLine2DAndCircle(circleRadius,
                                                circlePosition.getX(),
                                                circlePosition.getY(),
                                                startPoint.getX(),
                                                startPoint.getY(),
                                                false,
                                                pointOnRay.getX(),
                                                pointOnRay.getY(),
                                                true,
                                                firstIntersectionToPack,
                                                secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the intersection between a plane and an infinitely long line.
    * <a href="https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection"> Useful link </a>.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the line is parallel to the plane, this methods fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointOnPlaneX       x-value of a point located on the plane.
    * @param pointOnPlaneY       y-value of a point located on the plane.
    * @param pointOnPlaneZ       z-value of a point located on the plane.
    * @param planeNormalX       x-value of the normal of the plane.
    * @param planeNormalY       y-value of the normal of the plane.
    * @param planeNormalZ       z-value of the normal of the plane.
    * @param pointOnLineX       x-value of a point located on the line.
    * @param pointOnLineY       y-value of a point located on the line.
    * @param pointOnLineZ       z-value of a point located on the line.
    * @param lineDirectionX       x-value of the direction of the line.
    * @param lineDirectionY       y-value of the direction of the line.
    * @param lineDirectionZ       z-value of the direction of the line.
    * @param intersectionToPack point in which the coordinates of the intersection are stored. Modified.
    * @return {@code true} if the method succeeds, {@code false} otherwise.
    */
   public static boolean intersectionBetweenLine3DAndPlane3D(double pointOnPlaneX,
                                                             double pointOnPlaneY,
                                                             double pointOnPlaneZ,
                                                             double planeNormalX,
                                                             double planeNormalY,
                                                             double planeNormalZ,
                                                             double pointOnLineX,
                                                             double pointOnLineY,
                                                             double pointOnLineZ,
                                                             double lineDirectionX,
                                                             double lineDirectionY,
                                                             double lineDirectionZ,
                                                             Point3DBasics intersectionToPack)
   {
      // Switching to the notation used in https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection
      // Note: the algorithm is independent from the magnitudes of planeNormal and lineDirection

      // Let's compute the value of the coefficient d = ( (p0 - l0).n ) / ( l.n )
      double numerator, denominator;
      numerator = (pointOnPlaneX - pointOnLineX) * planeNormalX;
      numerator += (pointOnPlaneY - pointOnLineY) * planeNormalY;
      numerator += (pointOnPlaneZ - pointOnLineZ) * planeNormalZ;
      denominator = planeNormalX * lineDirectionX + planeNormalY * lineDirectionY + planeNormalZ * lineDirectionZ;

      // Check if the line is parallel to the plane
      if (Math.abs(denominator) < ONE_TRILLIONTH)
      {
         return false;
      }
      else
      {
         double d = numerator / denominator;

         intersectionToPack.setX(d * lineDirectionX + pointOnLineX);
         intersectionToPack.setY(d * lineDirectionY + pointOnLineY);
         intersectionToPack.setZ(d * lineDirectionZ + pointOnLineZ);
         return true;
      }
   }

   /**
    * This is only included here because it's a private method in the euclid class
    */
   public static boolean intersectionBetweenTwoLine2DsImpl(double start1x,
                                                           double start1y,
                                                           boolean canIntersectionOccurBeforeStart1,
                                                           double end1x,
                                                           double end1y,
                                                           boolean canIntersectionOccurBeforeEnd1,
                                                           double start2x,
                                                           double start2y,
                                                           boolean canIntersectionOccurBeforeStart2,
                                                           double end2x,
                                                           double end2y,
                                                           boolean canIntersectionOccurBeforeEnd2,
                                                           Point2DBasics intersectionToPack)
   {
      double epsilon = EuclidGeometryTools.ONE_TEN_MILLIONTH;

      double direction1x = end1x - start1x;
      double direction1y = end1y - start1y;
      double direction2x = end2x - start2x;
      double direction2y = end2y - start2y;

      double determinant = -direction1x * direction2y + direction1y * direction2x;

      double zeroish = 0.0 - epsilon;

      if (Math.abs(determinant) < epsilon)
      { // The lines are parallel
         // Check if they are collinear
         double dx = start2x - start1x;
         double dy = start2y - start1y;
         double cross = dx * direction1y - dy * direction1x;

         if (Math.abs(cross) < epsilon)
         {
            if (canIntersectionOccurBeforeStart1 && canIntersectionOccurBeforeEnd1)
            { // (start1, end1) represents a line
               if (canIntersectionOccurBeforeStart2 && canIntersectionOccurBeforeEnd2)
               { // (start2, end2) represents a line
                  if (intersectionToPack != null)
                     intersectionToPack.set(start1x, start1y);
                  return true;
               }

               if (intersectionToPack != null)
                  intersectionToPack.set(start2x, start2y);
               return true;
            }

            if (canIntersectionOccurBeforeStart2 && canIntersectionOccurBeforeEnd2)
            { // (start2, end2) represents a line
               if (intersectionToPack != null)
                  intersectionToPack.set(start1x, start1y);
               return true;
            }

            // Let's find the first endpoint that is inside the other line segment and return it.
            double direction1LengthSquare = EuclidCoreTools.normSquared(direction1x, direction1y);
            double dot;

            // Check if start2 is inside (start1, end1)
            dx = start2x - start1x;
            dy = start2y - start1y;
            dot = dx * direction1x + dy * direction1y;

            if ((canIntersectionOccurBeforeStart1 || zeroish < dot) && (canIntersectionOccurBeforeEnd1 || dot < direction1LengthSquare + epsilon))
            {
               if (intersectionToPack != null)
                  intersectionToPack.set(start2x, start2y);
               return true;
            }

            // Check if end2 is inside (start1, end1)
            dx = end2x - start1x;
            dy = end2y - start1y;
            dot = dx * direction1x + dy * direction1y;

            if ((canIntersectionOccurBeforeStart1 || zeroish < dot) && (canIntersectionOccurBeforeEnd1 || dot < direction1LengthSquare + epsilon))
            {
               if (intersectionToPack != null)
                  intersectionToPack.set(end2x, end2y);
               return true;
            }

            double direction2LengthSquare = EuclidCoreTools.normSquared(direction2x, direction2y);

            // Check if start1 is inside (start2, end2)
            dx = start1x - start2x;
            dy = start1y - start2y;
            dot = dx * direction2x + dy * direction2y;

            if ((canIntersectionOccurBeforeStart2 || zeroish < dot) && (canIntersectionOccurBeforeEnd2 || dot < direction2LengthSquare + epsilon))
            {
               if (intersectionToPack != null)
                  intersectionToPack.set(start1x, start1y);
               return true;
            }

            // Check if end1 is inside (start2, end2)
            dx = end1x - start2x;
            dy = end1y - start2y;
            dot = dx * direction2x + dy * direction2y;

            if ((canIntersectionOccurBeforeStart2 || zeroish < dot) && (canIntersectionOccurBeforeEnd2 || dot < direction2LengthSquare + epsilon))
            {
               if (intersectionToPack != null)
                  intersectionToPack.set(end1x, end1y);
               return true;
            }

            // (start1, end1) and (start2, end2) represent ray and/or line segment and they are collinear but do not overlap.
            if (intersectionToPack != null)
               intersectionToPack.setToNaN();
            return false;
         }
         // The lines are parallel but are not collinear, they do not intersect.
         else
         {
            if (intersectionToPack != null)
               intersectionToPack.setToNaN();
            return false;
         }
      }

      double dx = start2x - start1x;
      double dy = start2y - start1y;

      double oneOverDeterminant = 1.0 / determinant;
      double AInverse00 = -direction2y;
      double AInverse01 = direction2x;
      double AInverse10 = -direction1y;
      double AInverse11 = direction1x;

      double alpha = oneOverDeterminant * (AInverse00 * dx + AInverse01 * dy);
      double beta = oneOverDeterminant * (AInverse10 * dx + AInverse11 * dy);

      double oneish = 1.0 + epsilon;

      boolean areIntersecting = (canIntersectionOccurBeforeStart1 || zeroish < alpha) && (canIntersectionOccurBeforeEnd1 || alpha < oneish);
      if (areIntersecting)
         areIntersecting = (canIntersectionOccurBeforeStart2 || zeroish < beta) && (canIntersectionOccurBeforeEnd2 || beta < oneish);

      if (intersectionToPack != null)
      {
         if (areIntersecting)
         {
            intersectionToPack.set(start1x + alpha * direction1x, start1y + alpha * direction1y);
         }
         else
         {
            intersectionToPack.setToNaN();
         }
      }

      return areIntersecting;
   }

   /**
    * Just a more thorough version
    */
   public static boolean intersectionBetweenRay2DAndLine2D(double rayOriginX,
                                                           double rayOriginY,
                                                           double rayDirectionX,
                                                           double rayDirectionY,
                                                           double lineOriginX,
                                                           double lineOriginY,
                                                           double lineDirectionX,
                                                           double lineDirectionY,
                                                           Point2DBasics intersectionToPack)
   {
      double start1x = rayOriginX;
      double start1y = rayOriginY;
      double end1x = rayOriginX + rayDirectionX;
      double end1y = rayOriginY + rayDirectionY;
      double start2x = lineOriginX;
      double start2y = lineOriginY;
      double end2x = lineOriginX + lineDirectionX;
      double end2y = lineOriginY + lineDirectionY;
      return intersectionBetweenTwoLine2DsImpl(start1x, start1y, false, end1x, end1y, true, start2x, start2y, true, end2x, end2y, true, intersectionToPack);
   }

   public static boolean intersectionBetweenRay2DAndLine2D(Point2DReadOnly rayOrigin,
                                                           Vector2DReadOnly rayDirection,
                                                           Point2DReadOnly linePoint1,
                                                           Point2DReadOnly linePoint2,
                                                           Point2DBasics intersectionToPack)
   {
      return intersectionBetweenRay2DAndLine2D(rayOrigin.getX(),
                                               rayOrigin.getY(),
                                               rayDirection.getX(),
                                               rayDirection.getY(),
                                               linePoint1.getX(),
                                               linePoint1.getY(),
                                               linePoint2.getX() - linePoint1.getX(),
                                               linePoint2.getY() - linePoint1.getY(),
                                               intersectionToPack);
   }

   public static boolean intersectionBetweenRay2DAndLine2D(Point2DReadOnly rayOrigin,
                                                           Vector2DReadOnly rayDirection,
                                                           Point2DReadOnly lineOrigin,
                                                           Vector2DReadOnly lineDirection,
                                                           Point2DBasics intersectionToPack)
   {
      return intersectionBetweenRay2DAndLine2D(rayOrigin.getX(),
                                               rayOrigin.getY(),
                                               rayDirection.getX(),
                                               rayDirection.getY(),
                                               lineOrigin.getX(),
                                               lineOrigin.getY(),
                                               lineDirection.getX(),
                                               lineDirection.getY(),
                                               intersectionToPack);
   }

   /**
    * Computes the intersection between two infinitely long 2D lines each defined by a 2D point and a
    * 2D direction and returns a percentage {@code alpha} along the first line such that the
    * intersection coordinates can be computed as follows: <br>
    * {@code intersection = pointOnLine1 + alpha * lineDirection1}
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the two lines are parallel but not collinear, the two lines do not intersect and the
    * returned value is {@link Double#NaN}.
    * <li>if the two lines are collinear, the two lines are assumed to be intersecting infinitely the
    * returned value {@code Double.POSITIVE_INFINITY}.
    * </ul>
    * </p>
    *
    * @param startPoint1x    x-coordinate of a point located on the first line.
    * @param startPoint1y    y-coordinate of a point located on the first line.
    * @param segmentTravel1x x-component of the first line direction.
    * @param segmentTravel1y y-component of the first line direction.
    * @param startPoint2x    x-coordinate of a point located on the second line.
    * @param startPoint2y    y-coordinate of a point located on the second line.
    * @param segmentTravel2x x-component of the second line direction.
    * @param segmentTravel2y y-component of the second line direction.
    * @return {@code alpha} the percentage along the first line of the intersection location. This
    *       method returns {@link Double#NaN} if the lines do not intersect.
    */
   public static double percentageOfIntersectionBetweenTwoLine2DsInfCase(double startPoint1x,
                                                                         double startPoint1y,
                                                                         double segmentTravel1x,
                                                                         double segmentTravel1y,
                                                                         double startPoint2x,
                                                                         double startPoint2y,
                                                                         double segmentTravel2x,
                                                                         double segmentTravel2y)
   {
      //      We solve for x the problem of the form: A * x = b
      //            A      *     x     =      b
      //      / segmentTravel1x -segmentTravel2x \   / alpha \   / startPoint2x - startPoint1x \
      //      |                                  | * |       | = |                               |
      //      \ segmentTravel1y -segmentTravel2y /   \ beta  /   \ startPoint2y - startPoint1y /
      // Here, only alpha or beta is needed.

      double determinant = -segmentTravel1x * segmentTravel2y + segmentTravel1y * segmentTravel2x;

      double dx = startPoint2x - startPoint1x;
      double dy = startPoint2y - startPoint1y;

      if (Math.abs(determinant) < ONE_TRILLIONTH)
      { // The lines are parallel
         // Check if they are collinear
         double cross = dx * segmentTravel1y - dy * segmentTravel1x;
         if (Math.abs(cross) < ONE_TRILLIONTH)
         {
            /*
             * The two lines are collinear. There's an infinite number of intersection. Let's set the result to
             * infinity, i.e. alpha = infinity so it can be handled.
             */
            return Double.POSITIVE_INFINITY;
         }
         else
         {
            return Double.NaN;
         }
      }
      else
      {
         double oneOverDeterminant = 1.0 / determinant;
         double AInverse00 = oneOverDeterminant * -segmentTravel2y;
         double AInverse01 = oneOverDeterminant * segmentTravel2x;

         double alpha = AInverse00 * dx + AInverse01 * dy;

         return alpha;
      }
   }

   public static double getZOnPlane(Point3DReadOnly pointOnPlane, Vector3DReadOnly planeNormal, double pointX, double pointY)
   {
      // The three components of the plane origin
      double x0 = pointOnPlane.getX();
      double y0 = pointOnPlane.getY();
      double z0 = pointOnPlane.getZ();
      // The three components of the plane normal
      double a = planeNormal.getX();
      double b = planeNormal.getY();
      double c = planeNormal.getZ();

      // Given the plane equation: a*x + b*y + c*z + d = 0, with d = -(a*x0 + b*y0 + c*z0), we find z:
      double z = a / c * (x0 - pointX) + b / c * (y0 - pointY) + z0;
      return z;
   }

   /**
    * Finds the intersection of two bounding boxes defined by a bounding box Allocates a new
    * BoundingBox2D. TODO: Check, Unit test, move where BoundingBox union is
    *
    * @param a
    * @param b
    * @return the intersection bounding box, or null if no intersection
    */
   public static BoundingBox2D computeIntersectionOfTwoBoundingBoxes(BoundingBox2DReadOnly a, BoundingBox2DReadOnly b)
   {
      double maxX = Math.min(a.getMaxX(), b.getMaxX());
      double maxY = Math.min(a.getMaxY(), b.getMaxY());
      double minX = Math.max(a.getMinX(), b.getMinX());
      double minY = Math.max(a.getMinY(), b.getMinY());

      if ((maxX <= minX) || (maxY <= minY))
         return null;

      return new BoundingBox2D(minX, minY, maxX, maxY);
   }

   /**
    * Finds the intersection of two bounding boxes defined by a bounding box Allocates a new boundingBox3D.
    *
    * @param a
    * @param b
    * @return the intersection bounding box, or null if no intersection
    */
   public static BoundingBox3D computeIntersectionOfTwoBoundingBoxes(BoundingBox3DReadOnly a, BoundingBox3DReadOnly b)
   {
      double maxX = Math.min(a.getMaxX(), b.getMaxX());
      double maxY = Math.min(a.getMaxY(), b.getMaxY());
      double maxZ = Math.min(a.getMaxZ(), b.getMaxZ());

      double minX = Math.max(a.getMinX(), b.getMinX());
      double minY = Math.max(a.getMinY(), b.getMinY());
      double minZ = Math.max(a.getMinZ(), b.getMinZ());

      if ((maxX <= minX) || (maxY <= minY) || (maxZ <= minZ))
         return null;

      return new BoundingBox3D(minX, minY, minZ, maxX, maxY, maxZ);
   }

   public static double computeBoundingBoxVolume3D(BoundingBox3DReadOnly boundingBox)
   {
      return Math.abs(boundingBox.getMaxX() - boundingBox.getMinX()) * Math.abs(boundingBox.getMaxY() - boundingBox.getMinY()) * Math.abs(
            boundingBox.getMaxZ() - boundingBox.getMinZ());
   }

   public static boolean doLineSegment2DAndConvexPolygon2DIntersect(Point2DReadOnly lineSegmentStart,
                                                                    Point2DReadOnly lineSegmentEnd,
                                                                    List<? extends Point2DReadOnly> convexPolygon2D,
                                                                    int numberOfVertices)
   {
      checkNumberOfVertices(convexPolygon2D, numberOfVertices);

      if (numberOfVertices == 0)
         return false;

      if (numberOfVertices == 1)
         return EuclidGeometryTools.isPoint2DOnLineSegment2D(convexPolygon2D.get(0), lineSegmentStart, lineSegmentEnd);

      if (numberOfVertices == 2)
         return EuclidGeometryTools.doLineSegment2DsIntersect(convexPolygon2D.get(0), convexPolygon2D.get(1), lineSegmentStart, lineSegmentEnd);

      for (int edgeIndex = 0; edgeIndex < numberOfVertices; edgeIndex++)
      {
         Point2DReadOnly edgeStart = convexPolygon2D.get(edgeIndex);
         Point2DReadOnly edgeEnd = convexPolygon2D.get(EuclidGeometryPolygonTools.next(edgeIndex, numberOfVertices));

         if (EuclidGeometryTools.doLineSegment2DsIntersect(edgeStart, edgeEnd, lineSegmentStart, lineSegmentEnd))
            return true;
      }

      return false;
   }

   private static void checkNumberOfVertices(List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices)
   {
      if (numberOfVertices < 0 || numberOfVertices > convexPolygon2D.size())
         throw new IllegalArgumentException("Illegal numberOfVertices: " + numberOfVertices + ", expected a value in ] 0, " + convexPolygon2D.size() + "].");
   }

   /**
    * Compute Intersection-over-Union (IoU) of two 3D bounding boxes.
    */
   public static double computeIntersectionOverUnionOfTwoBoundingBoxes(BoundingBox3DReadOnly a, BoundingBox3DReadOnly b)
   {
      BoundingBox3D intersection = computeIntersectionOfTwoBoundingBoxes(a, b);

      if (intersection == null)
         return 0.0;

      double intersectionVolume = computeBoundingBoxVolume3D(intersection);
      double volumeA = computeBoundingBoxVolume3D(a);
      double volumeB = computeBoundingBoxVolume3D(b);
      double unionVolume = volumeA + volumeB - intersectionVolume;

      return intersectionVolume / unionVolume;
   }

   /**
    * Compute Intersection-over-Union (IoU) of two 3D bounding boxes.
    */
   public static double computeIntersectionOverSmallerOfTwoBoundingBoxes(BoundingBox3DReadOnly a, BoundingBox3DReadOnly b)
   {
      BoundingBox3D intersection = computeIntersectionOfTwoBoundingBoxes(a, b);

      if (intersection == null)
         return 0.0;

      double intersectionVolume = computeBoundingBoxVolume3D(intersection);
      double volumeA = computeBoundingBoxVolume3D(a);
      double volumeB = computeBoundingBoxVolume3D(b);
      double smallerVolume = Math.min(volumeA, volumeB);

      return intersectionVolume / smallerVolume;
   }

   /**
    * Determines if the polygonToTest is inside the convex polygon.
    */
   public static boolean isPolygonInside(ConvexPolygon2DReadOnly polygonToTest, double epsilon, ConvexPolygon2DReadOnly polygon)
   {
      for (int i = 0; i < polygonToTest.getNumberOfVertices(); i++)
      {
         if (!polygon.isPointInside(polygonToTest.getVertex(i), epsilon))
            return false;
      }

      return true;
   }

   /**
    * Determines if the polygonToTest is inside the convex polygon.
    */
   public static boolean isPolygonInside(ConvexPolygon2DReadOnly polygonToTest, ConvexPolygon2DReadOnly polygon)
   {
      return isPolygonInside(polygonToTest, 0.0, polygon);
   }

   /**
    * Finds the projection of a 3D point onto a 3D plane given in general form.
    * Uses: projectedPoint = point - (normal.dot(point) + planeScalar) * (normal)
    *
    * @param plane Coefficients of the general form of plane equation (ax + by + cz + d = 0) as Vector4D
    * @param point Point to be projected onto the plane as Point3D
    * @return Projected point onto the plane as Point3D
    */
   public static Point3D projectPointOntoPlane(Vector4DReadOnly plane, Point3DReadOnly point)
   {
      UnitVector3D planeNormal = new UnitVector3D(plane.getX(), plane.getY(), plane.getZ());

      Vector3D scaledNormal = new Vector3D(planeNormal);
      scaledNormal.scale(planeNormal.dot(point) + plane.getS());

      Point3D projectedPoint = new Point3D();
      projectedPoint.sub(point, scaledNormal);

      return projectedPoint;
   }
}
