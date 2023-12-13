package us.ihmc.robotics;

import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.ONE_MILLIONTH;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.ONE_TRILLIONTH;
import static us.ihmc.euclid.tools.EuclidCoreRandomTools.nextDouble;
import static us.ihmc.euclid.tools.EuclidCoreRandomTools.nextMatrix3D;
import static us.ihmc.euclid.tools.EuclidCoreTools.normSquared;

import java.lang.reflect.Field;
import java.util.Random;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.CommonMatrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameTuple3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
public class EuclidCoreMissingTools
{
   public static final String DEGREE_SYMBOL = "\u00B0";

   public static void transform(Matrix3DReadOnly matrix, double xOriginal, double yOriginal, double zOriginal, Tuple3DBasics tupleTransformed)
   {
      double x = matrix.getM00() * xOriginal + matrix.getM01() * yOriginal + matrix.getM02() * zOriginal;
      double y = matrix.getM10() * xOriginal + matrix.getM11() * yOriginal + matrix.getM12() * zOriginal;
      double z = matrix.getM20() * xOriginal + matrix.getM21() * yOriginal + matrix.getM22() * zOriginal;
      tupleTransformed.set(x, y, z);
   }

   public static void floorToGivenPrecision(Tuple3DBasics tuple3d, double precision)
   {
      tuple3d.setX(MathTools.floorToPrecision(tuple3d.getX(), precision));
      tuple3d.setY(MathTools.floorToPrecision(tuple3d.getY(), precision));
      tuple3d.setZ(MathTools.floorToPrecision(tuple3d.getZ(), precision));
   }

   public static void roundToGivenPrecision(Tuple3DBasics tuple3d, double precision)
   {
      tuple3d.setX(MathTools.roundToPrecision(tuple3d.getX(), precision));
      tuple3d.setY(MathTools.roundToPrecision(tuple3d.getY(), precision));
      tuple3d.setZ(MathTools.roundToPrecision(tuple3d.getZ(), precision));
   }

   public static void floorToGivenPrecision(Tuple2DBasics tuple2d, double precision)
   {
      tuple2d.setX(MathTools.floorToPrecision(tuple2d.getX(), precision));
      tuple2d.setY(MathTools.floorToPrecision(tuple2d.getY(), precision));
   }

   public static void roundToGivenPrecision(Tuple2DBasics tuple2d, double precision)
   {
      tuple2d.setX(MathTools.roundToPrecision(tuple2d.getX(), precision));
      tuple2d.setY(MathTools.roundToPrecision(tuple2d.getY(), precision));
   }

   public static boolean isFinite(Tuple3DBasics tuple)
   {
      return Double.isFinite(tuple.getX()) && Double.isFinite(tuple.getY()) && Double.isFinite(tuple.getZ());
   }

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

   /**
    * Projects the provided {@code rotation} onto {@code axis} such that the original rotation can be
    * decomposed into a rotation around {@code axis} and one around an orthogonal axis.
    * <p>
    * rotation = orthogonalRotation * result
    * </p>
    *
    * @param rotation is the original rotation to be projected onto {@code axis}
    * @param axis     is the desired rotation axis of the result.
    * @param result   will be modified to contain the component of {@code rotation} that is around
    *                 {@code axis}
    */
   public static void projectRotationOnAxis(QuaternionReadOnly rotation, Vector3DReadOnly axis, QuaternionBasics result)
   {
      double dotProduct = rotation.getX() * axis.getX() + rotation.getY() * axis.getY() + rotation.getZ() * axis.getZ();

      double scale = dotProduct / axis.normSquared();
      double projectedX = scale * axis.getX();
      double projectedY = scale * axis.getY();
      double projectedZ = scale * axis.getZ();

      result.set(projectedX, projectedY, projectedZ, rotation.getS());
      result.normalize();
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
    *         method returns {@link Double#NaN} if the lines do not intersect.
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

      if (Math.abs(determinant) < EuclidGeometryTools.ONE_TRILLIONTH)
      { // The lines are parallel
        // Check if they are collinear
         double cross = dx * segmentTravel1y - dy * segmentTravel1x;
         if (Math.abs(cross) < EuclidGeometryTools.ONE_TRILLIONTH)
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
    *      Vector3DReadOnly, Orientation3DBasics)
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
    *      double, double, Orientation3DBasics)
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

   public static double distanceSquaredFromPoint2DToLineSegment2D(double pointX,
                                                                  double pointY,
                                                                  double lineSegmentStartX,
                                                                  double lineSegmentStartY,
                                                                  double lineSegmentEndX,
                                                                  double lineSegmentEndY,
                                                                  Point2D intersectionPointToPack)
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

      if (intersectionPointToPack != null)
         intersectionPointToPack.set(projectionX, projectionY);

      double dx = projectionX - pointX;
      double dy = projectionY - pointY;
      return dx * dx + dy * dy;
   }

   /**
    * Calculates the 3D part of the given {@code input} that is parallel to the given
    * {@code normalAxis} and stores the result in {@code inputNormalPartToPack}.
    * <p>
    * The result has the following properties:
    * <ul>
    * <li><tt>x.n = x<sub>n</sub>.n</tt>
    * <li><tt>|x<sub>n</sub>&times;n| = 0</tt>
    * </ul>
    * where:
    * <ul>
    * <li><tt>x</tt> is {@code input}.
    * <li></tt>n</tt> is {@code normalAxis}.
    * <li><tt>x<sub>n</sub></tt> is {@code inputNormalPartToPack}.
    * </ul>
    * </p>
    *
    * @param input                 the tuple to extract the normal part of. Not modified.
    * @param normalAxis            the normal vector. It is normalized internally if needed. Not
    *                              modified.
    * @param inputNormalPartToPack the tuple used to store the normal part of the input. Modified.
    */
   public static void extractNormalPart(Tuple3DReadOnly input, Vector3DReadOnly normalAxis, Tuple3DBasics inputNormalPartToPack)
   {
      double normalX = normalAxis.getX();
      double normalY = normalAxis.getY();
      double normalZ = normalAxis.getZ();
      double normalLengthSquared = EuclidCoreTools.normSquared(normalX, normalY, normalZ);

      double dot = TupleTools.dot(normalAxis, input) / normalLengthSquared;
      inputNormalPartToPack.setAndScale(dot, normalAxis);
   }

   /**
    * Calculates the 3D part of the given {@code input} that is parallel to the given
    * {@code normalAxis} and stores the result in {@code inputNormalPartToPack}.
    * <p>
    * The result has the following properties:
    * <ul>
    * <li><tt>x.n = x<sub>n</sub>.n</tt>
    * <li><tt>|x<sub>n</sub>&times;n| = 0</tt>
    * </ul>
    * where:
    * <ul>
    * <li><tt>x</tt> is {@code input}.
    * <li></tt>n</tt> is {@code normalAxis}.
    * <li><tt>x<sub>n</sub></tt> is {@code inputNormalPartToPack}.
    * </ul>
    * </p>
    *
    * @param input                 the tuple to extract the normal part of. Not modified.
    * @param normalAxis            the normal vector. It is normalized internally if needed. Not
    *                              modified.
    * @param inputNormalPartToPack the tuple used to store the normal part of the input. Modified.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame.
    */
   public static void extractNormalPart(FrameTuple3DReadOnly input, FrameVector3DReadOnly normalAxis, FixedFrameTuple3DBasics inputNormalPartToPack)
   {
      input.checkReferenceFrameMatch(normalAxis, inputNormalPartToPack);
      extractNormalPart((Tuple3DReadOnly) input, (Vector3DReadOnly) normalAxis, (Tuple3DBasics) inputNormalPartToPack);
   }

   /**
    * Calculates the 3D part of the given {@code input} that is parallel to the given
    * {@code normalAxis} and stores the result in {@code inputNormalPartToPack}.
    * <p>
    * The result has the following properties:
    * <ul>
    * <li><tt>x.n = x<sub>n</sub>.n</tt>
    * <li><tt>|x<sub>n</sub>&times;n| = 0</tt>
    * </ul>
    * where:
    * <ul>
    * <li><tt>x</tt> is {@code input}.
    * <li></tt>n</tt> is {@code normalAxis}.
    * <li><tt>x<sub>n</sub></tt> is {@code inputNormalPartToPack}.
    * </ul>
    * </p>
    *
    * @param input                 the tuple to extract the normal part of. Not modified.
    * @param normalAxis            the normal vector. It is normalized internally if needed. Not
    *                              modified.
    * @param inputNormalPartToPack the tuple used to store the normal part of the input. Modified.
    * @throws ReferenceFrameMismatchException if {@code input} and {@code normalAxis} are not expressed
    *                                         in the same reference frame.
    */
   public static void extractNormalPart(FrameTuple3DReadOnly input, FrameVector3DReadOnly normalAxis, FrameTuple3DBasics inputNormalPartToPack)
   {
      input.checkReferenceFrameMatch(normalAxis);
      inputNormalPartToPack.setReferenceFrame(input.getReferenceFrame());
      extractNormalPart((Tuple3DReadOnly) input, (Vector3DReadOnly) normalAxis, (Tuple3DBasics) inputNormalPartToPack);
   }

   /**
    * Calculates the 3D part of the given {@code input} that is orthogonal to the given
    * {@code normalAxis} and stores the result in {@code inputTangentialPartToPack}.
    * <p>
    * The result has the following properties:
    * <ul>
    * <li><tt>x<sub>t</sub>.n = 0</tt>
    * <li><tt>|x - (x.n)n| = |x<sub>t</sub>|</tt>
    * </ul>
    * where:
    * <ul>
    * <li><tt>x</tt> is {@code input}.
    * <li></tt>n</tt> is {@code normalAxis}.
    * <li><tt>x<sub>t</sub></tt> is {@code inputTangentialPartToPack}.
    * </ul>
    * </p>
    *
    * @param input                     the tuple to extract the tangential part of. Not modified.
    * @param normalAxis                the normal vector. It is normalized internally if needed. Not
    *                                  modified.
    * @param inputTangentialPartToPack the tuple used to store the tangential part of the input.
    *                                  Modified.
    */
   public static void extractTangentialPart(Tuple3DReadOnly input, Vector3DReadOnly normalAxis, Tuple3DBasics inputTagentialPartToPack)
   {
      double normalX = normalAxis.getX();
      double normalY = normalAxis.getY();
      double normalZ = normalAxis.getZ();
      double normalLengthSquared = EuclidCoreTools.normSquared(normalX, normalY, normalZ);

      double dot = TupleTools.dot(normalX, normalY, normalZ, input) / normalLengthSquared;
      double normalPartX = dot * normalX;
      double normalPartY = dot * normalY;
      double normalPartZ = dot * normalZ;

      inputTagentialPartToPack.set(input);
      inputTagentialPartToPack.sub(normalPartX, normalPartY, normalPartZ);
   }

   /**
    * Calculates the 3D part of the given {@code input} that is orthogonal to the given
    * {@code normalAxis} and stores the result in {@code inputTangentialPartToPack}.
    * <p>
    * The result has the following properties:
    * <ul>
    * <li><tt>x<sub>t</sub>.n = 0</tt>
    * <li><tt>|x - (x.n)n| = |x<sub>t</sub>|</tt>
    * </ul>
    * where:
    * <ul>
    * <li><tt>x</tt> is {@code input}.
    * <li></tt>n</tt> is {@code normalAxis}.
    * <li><tt>x<sub>t</sub></tt> is {@code inputTangentialPartToPack}.
    * </ul>
    * </p>
    *
    * @param input                     the tuple to extract the tangential part of. Not modified.
    * @param normalAxis                the normal vector. It is normalized internally if needed. Not
    *                                  modified.
    * @param inputTangentialPartToPack the tuple used to store the tangential part of the input.
    *                                  Modified.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame.
    */
   public static void extractTangentialPart(FrameTuple3DReadOnly input, FrameVector3DReadOnly normalAxis, FixedFrameTuple3DBasics inputTangentialPartToPack)
   {
      input.checkReferenceFrameMatch(normalAxis, inputTangentialPartToPack);
      extractTangentialPart((Tuple3DReadOnly) input, (Vector3DReadOnly) normalAxis, (Tuple3DBasics) inputTangentialPartToPack);
   }

   /**
    * Calculates the 3D part of the given {@code input} that is orthogonal to the given
    * {@code normalAxis} and stores the result in {@code inputTangentialPartToPack}.
    * <p>
    * The result has the following properties:
    * <ul>
    * <li><tt>x<sub>t</sub>.n = 0</tt>
    * <li><tt>|x - (x.n)n| = |x<sub>t</sub>|</tt>
    * </ul>
    * where:
    * <ul>
    * <li><tt>x</tt> is {@code input}.
    * <li></tt>n</tt> is {@code normalAxis}.
    * <li><tt>x<sub>t</sub></tt> is {@code inputTangentialPartToPack}.
    * </ul>
    * </p>
    *
    * @param input                     the tuple to extract the tangential part of. Not modified.
    * @param normalAxis                the normal vector. It is normalized internally if needed. Not
    *                                  modified.
    * @param inputTangentialPartToPack the tuple used to store the tangential part of the input.
    *                                  Modified.
    * @throws ReferenceFrameMismatchException if {@code input} and {@code normalAxis} are not expressed
    *                                         in the same reference frame.
    */
   public static void extractTangentialPart(FrameTuple3DReadOnly input, FrameVector3DReadOnly normalAxis, FrameTuple3DBasics inputTangentialPartToPack)
   {
      input.checkReferenceFrameMatch(normalAxis);
      inputTangentialPartToPack.setReferenceFrame(input.getReferenceFrame());
      extractTangentialPart((Tuple3DReadOnly) input, (Vector3DReadOnly) normalAxis, (Tuple3DBasics) inputTangentialPartToPack);
   }

   /**
    * Modifies {@code tupleToModify} such that its normal part is equal to the normal part of
    * {@code input}.
    * <p>
    * This method performs the following calculation: <tt>x = x - (x.n)n + (y.n)n</tt>, where:
    * <ul>
    * <li><tt>x</tt> is {@code tupleToModify}.
    * <li></tt>n</tt> is {@code normalAxis}.
    * <li><tt>y</tt> is {@code input}.
    * </ul>
    * </p>
    *
    * @param input         the tuple containing the normal part used to update {@code tupleToModify}.
    *                      Not modified.
    * @param normalAxis    the normal vector. It is normalized internally if needed. Not modified.
    * @param tupleToModify the tuple to modify the normal part of. Modified.
    */
   public static void setNormalPart(Tuple3DReadOnly input, Vector3DReadOnly normalAxis, Tuple3DBasics tupleToModify)
   {
      double normalX = normalAxis.getX();
      double normalY = normalAxis.getY();
      double normalZ = normalAxis.getZ();
      double normalLengthSquared = EuclidCoreTools.normSquared(normalX, normalY, normalZ);
      double dot = (TupleTools.dot(normalAxis, input) - TupleTools.dot(normalAxis, tupleToModify)) / normalLengthSquared;
      tupleToModify.scaleAdd(dot, normalAxis, tupleToModify);
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

   private static int intersectionBetweenLine2DAndCircle(double circleRadius,
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

         if (Math.abs(EuclidGeometryTools.percentageAlongLine2D(intersection1X, intersection1Y, circlePositionX, circlePositionY, 1.0, 0.0)) > circleRadius
               - ONE_TRILLIONTH)
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

         if (Math.abs(EuclidGeometryTools.percentageAlongLine2D(intersection2X, intersection2Y, circlePositionX, circlePositionY, 1.0, 0.0)) > circleRadius
               - ONE_TRILLIONTH)
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

   /**
    * Computes the angle in radians from the first 3D vector to the second 3D vector. The computed
    * angle is in the range [0; <i>pi</i>].
    *
    * @param firstVector  the first vector. Not modified.
    * @param secondVector the second vector. Not modified.
    * @return the angle in radians from the first vector to the second vector.
    */
   public static double angleFromFirstToSecondVector3D(Vector3DReadOnly firstVector, Vector3DReadOnly secondVector)
   {
      return angleFromFirstToSecondVector3D(firstVector.getX(),
                                            firstVector.getY(),
                                            firstVector.getZ(),
                                            firstVector instanceof UnitVector3DReadOnly,
                                            secondVector.getX(),
                                            secondVector.getY(),
                                            secondVector.getZ(),
                                            secondVector instanceof UnitVector3DReadOnly);
   }

   private static double angleFromFirstToSecondVector3D(double firstVectorX,
                                                        double firstVectorY,
                                                        double firstVectorZ,
                                                        boolean isFirstVectorUnitary,
                                                        double secondVectorX,
                                                        double secondVectorY,
                                                        double secondVectorZ,
                                                        boolean isSecondVectorUnitary)
   {
      double firstVectorLength = isFirstVectorUnitary ? 1.0 : EuclidCoreTools.norm(firstVectorX, firstVectorY, firstVectorZ);
      double secondVectorLength = isSecondVectorUnitary ? 1.0 : EuclidCoreTools.norm(secondVectorX, secondVectorY, secondVectorZ);

      double dotProduct = firstVectorX * secondVectorX + firstVectorY * secondVectorY + firstVectorZ * secondVectorZ;
      dotProduct /= firstVectorLength * secondVectorLength;

      if (dotProduct > 1.0)
         dotProduct = 1.0;
      else if (dotProduct < -1.0)
         dotProduct = -1.0;

      return EuclidCoreTools.acos(dotProduct);
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
    * @param intersectionToPack point in which the coordinates of the intersection are stored.
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

   /**
    * This is only included here because it's a private method in the euclid class
    */
   private static boolean intersectionBetweenTwoLine2DsImpl(double start1x,
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
    * Calculate the angular velocity by differentiating orientation.
    * 
    * @param qStart                the initial orientation at time t.
    * @param qEnd                  the final orientation at time t + duration.
    * @param duration              the time interval between the 2 orientations.
    * @param angularVelocityToPack the angular velocity.
    */
   public static void differentiateOrientation(QuaternionReadOnly qStart, QuaternionReadOnly qEnd, double duration, Vector3DBasics angularVelocityToPack)
   {
      double q1x = qStart.getX();
      double q1y = qStart.getY();
      double q1z = qStart.getZ();
      double q1s = qStart.getS();

      double q2x = qEnd.getX();
      double q2y = qEnd.getY();
      double q2z = qEnd.getZ();
      double q2s = qEnd.getS();

      double diffx = q1s * q2x - q1x * q2s - q1y * q2z + q1z * q2y;
      double diffy = q1s * q2y + q1x * q2z - q1y * q2s - q1z * q2x;
      double diffz = q1s * q2z - q1x * q2y + q1y * q2x - q1z * q2s;
      double diffs = q1s * q2s + q1x * q2x + q1y * q2y + q1z * q2z;

      if (diffs < 0.0)
      {
         diffx = -diffx;
         diffy = -diffy;
         diffz = -diffz;
         diffs = -diffs;
      }

      double sinHalfTheta = EuclidCoreTools.norm(diffx, diffy, diffz);

      double angle;
      if (EuclidCoreTools.epsilonEquals(1.0, diffs, 1.0e-12))
         angle = 2.0 * sinHalfTheta / diffs;
      else
         angle = 2.0 * EuclidCoreTools.atan2(sinHalfTheta, diffs);
      angularVelocityToPack.set(diffx, diffy, diffz);
      angularVelocityToPack.scale(angle / (sinHalfTheta * duration));
   }

   // *** NOTE ***: The 4x4 output matrix produced by this method assumes a Quaternion component ordering of:
   //   Quat = [ Qs
   //            Qx
   //            Qy
   //            Qz ]
   public static DMatrixRMaj quaternionDotToOmegaTransform(QuaternionReadOnly rotatingFrameQuaternion)
   {
      double qs = rotatingFrameQuaternion.getS();
      double qx = rotatingFrameQuaternion.getX();
      double qy = rotatingFrameQuaternion.getY();
      double qz = rotatingFrameQuaternion.getZ();

      DMatrixRMaj E = new DMatrixRMaj(4,4);

      E.set(0,0, qs); E.set(0,1, qx); E.set(0,2, qy); E.set(0,3, qz);
      E.set(1,0,-qx); E.set(1,1, qs); E.set(1,2, qz); E.set(1,3,-qy);
      E.set(2,0,-qy); E.set(2,1,-qz); E.set(2,2, qs); E.set(2,3, qx);
      E.set(3,0,-qz); E.set(3,1, qy); E.set(3,2,-qx); E.set(3,3, qs);
      
      return E;
   }

   /**
    * Sets the yaw pitch roll but the doubles are given in degrees.
    */
   public static void setYawPitchRollDegrees(Orientation3DBasics orientation3DBasics, double yaw, double pitch, double roll)
   {
      orientation3DBasics.setYawPitchRoll(Math.toRadians(yaw), Math.toRadians(pitch), Math.toRadians(roll));
   }

   /**
    * Get the orientation as yaw pitch roll String but they are in degrees.
    * Says yaw-pitch-roll.
    */
   public static String getYawPitchRollStringDegrees(Orientation3DBasics orientation3DBasics)
   {
      // Degree symbol placed at the end so you don't have to remove it when copy and pasting
      return EuclidCoreIOTools.getYawPitchRollString(EuclidCoreIOTools.DEFAULT_FORMAT,
                                                     Math.toDegrees(orientation3DBasics.getYaw()),
                                                     Math.toDegrees(orientation3DBasics.getPitch()),
                                                     Math.toDegrees(orientation3DBasics.getRoll())) + DEGREE_SYMBOL;
   }

   /**
    * Get the orientation as yaw pitch roll String but they are in degrees.
    * Doesn't say yaw-pitch-roll.
    */
   public static String getYawPitchRollValuesStringDegrees(Orientation3DBasics orientation3DBasics)
   {
      // Degree symbol placed at the end so you don't have to remove it when copy and pasting
      return EuclidCoreIOTools.getStringOf("(", ")", ", ",
                                           EuclidCoreIOTools.DEFAULT_FORMAT,
                                           Math.toDegrees(orientation3DBasics.getYaw()),
                                           Math.toDegrees(orientation3DBasics.getPitch()),
                                           Math.toDegrees(orientation3DBasics.getRoll())) + DEGREE_SYMBOL;
   }

   /**
    * Generates a random positive definite matrix.
    * <p>
    * {@code matrix}<sub>ij</sub> &in; [-1.0; 1.0].
    * </p>
    * <p>
    * The approach used here generates a random 3D matrix with values in [-1.0, 1.0], and then performs A * A<sup>T</sup> which is guaranteed to result in a
    * symmetric positive semi-definite matrix. We then add diagonal terms to make the matrix positive definite, and finally scale the matrix by a random double
    * that upper bounds the absolute values of the positive definite matrix elements to 1.0.
    * </p>
    *
    * @param random the random generator to use.
    * @return the random positive definite matrix.
    */
   public static Matrix3D nextPositiveDefiniteMatrix3D(Random random)
   {
      return nextPositiveDefiniteMatrix3D(random, 1.0);
   }

   /**
    * Generates a random positive definite matrix.
    * <p>
    * {@code matrix}<sub>ij</sub> &in; [-minMaxValue, minMaxValue]
    * </p>
    * <p>
    * The approach used here generates a random 3D matrix with values in [{@code -minMaxValue}, {@code minMaxValue}], and then performs A * A<sup>T</sup>,
    * which is guaranteed to result in a symmetric positive semi-definite matrix. We then add diagonal terms to make the matrix positive definite, and finally
    * scale the matrix by a random double that upper bounds the absolute values of the positive definite matrix elements to {@code minMaxValue}.
    * </p>
    *
    * @param random      the random generator to use.
    * @param minMaxValue the maximum value for each element.
    * @return the random positive definite matrix.
    * @throws RuntimeException if {@code minMaxValue < 0}.
    */
   public static Matrix3D nextPositiveDefiniteMatrix3D(Random random, double minMaxValue)
   {
      Matrix3D matrix3D = nextMatrix3D(random, minMaxValue);
      matrix3D.multiplyTransposeOther(matrix3D);

      double diagonalDominanceScalar = Math.abs(minMaxValue);
      matrix3D.addM00(diagonalDominanceScalar);
      matrix3D.addM11(diagonalDominanceScalar);
      matrix3D.addM22(diagonalDominanceScalar);

      double scalarToShrinkMatrixWithinBounds = nextDouble(random, 0.0, minMaxValue / matrix3D.maxAbsElement());
      matrix3D.scale(scalarToShrinkMatrixWithinBounds);
      return matrix3D;
   }

   /**
    * Remove when this issue is fixed:
    * https://github.com/ihmcrobotics/euclid/issues/57
    */
   private static final Field referenceFrameHasBeenRemoved;
   static
   {
      try
      {
         referenceFrameHasBeenRemoved = ReferenceFrame.class.getDeclaredField("hasBeenRemoved");
         referenceFrameHasBeenRemoved.setAccessible(true);
      }
      catch (NoSuchFieldException e)
      {
         throw new RuntimeException(e);
      }
   }

   public static boolean hasBeenRemoved(ReferenceFrame referenceFrame)
   {
      try
      {
         return referenceFrameHasBeenRemoved.getBoolean(referenceFrame);
      }
      catch (IllegalAccessException e)
      {
         throw new RuntimeException(e);
      }
   }

   /**
    * Remove when this issue is fixed:
    * https://github.com/ihmcrobotics/euclid/issues/57
    */
   private static final Field referenceFrameName;
   static
   {
      try
      {
         referenceFrameName = ReferenceFrame.class.getDeclaredField("frameName");
         referenceFrameName.setAccessible(true);
      }
      catch (NoSuchFieldException e)
      {
         throw new RuntimeException(e);
      }
   }

   public static String frameName(ReferenceFrame referenceFrame)
   {
      try
      {
         return referenceFrameName.get(referenceFrame).toString();
      }
      catch (IllegalAccessException e)
      {
         throw new RuntimeException(e);
      }
   }

   public static FrameVector3DReadOnly newLinkedFrameVector3DReadOnly(ReferenceFrameHolder referenceFrameHolder, DMatrixRMaj source)
   {
      return newLinkedFrameVector3DReadOnly(referenceFrameHolder, 0, source);
   }

   public static FrameVector3DReadOnly newLinkedFrameVector3DReadOnly(ReferenceFrameHolder referenceFrameHolder, int startIndex, DMatrixRMaj source)
   {
      int xIndex = startIndex;
      int yIndex = startIndex + 1;
      int zIndex = startIndex + 2;
      return EuclidFrameFactories.newLinkedFrameVector3DReadOnly(referenceFrameHolder,
                                                                 () -> source.get(xIndex, 0),
                                                                 () -> source.get(yIndex, 0),
                                                                 () -> source.get(zIndex, 0));
   }
}