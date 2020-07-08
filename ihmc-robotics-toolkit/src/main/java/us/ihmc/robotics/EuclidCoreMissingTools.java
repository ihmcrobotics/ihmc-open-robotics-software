package us.ihmc.robotics;

import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.ONE_MILLIONTH;
import static us.ihmc.euclid.tools.EuclidCoreTools.normSquared;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.CommonMatrix3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameTuple3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class EuclidCoreMissingTools
{

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
    * @param lineSegmentEnd the second endpoint of the line segment. Not modified.
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
    * @param axis is the desired rotation axis of the result.
    * @param result will be modified to contain the component of {@code rotation} that is around
    *       {@code axis}
    */
   public static void projectRotationOnAxis(QuaternionReadOnly rotation, Vector3DReadOnly axis, QuaternionBasics result)
   {
      double dotProduct = rotation.getX() * axis.getX() + rotation.getY() * axis.getY() + rotation.getZ() * axis.getZ();

      double scale = dotProduct / axis.lengthSquared();
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
    * <li>if the two lines are collinear, the two lines are assumed to be intersecting infinitely the returned value {@code Double.POSITIVE_INFINITY}.
    * </ul>
    * </p>
    *
    * @param startPoint1x x-coordinate of a point located on the first line.
    * @param startPoint1y y-coordinate of a point located on the first line.
    * @param segmentTravel1x x-component of the first line direction.
    * @param segmentTravel1y y-component of the first line direction.
    * @param startPoint2x x-coordinate of a point located on the second line.
    * @param startPoint2y y-coordinate of a point located on the second line.
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

      if (Math.abs(determinant) < EuclidGeometryTools.ONE_TRILLIONTH)
      { // The lines are parallel
         // Check if they are collinear
         double cross = dx * segmentTravel1y - dy * segmentTravel1x;
         if (Math.abs(cross) < EuclidGeometryTools.ONE_TRILLIONTH)
         {
            /*
             * The two lines are collinear. There's an infinite number of intersection. Let's set the
             * result to infinity, i.e. alpha = infinity so it can be handled.
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
    * @param vector the vector that is rotated with respect to {@code zUp}. Not modified.
    * @param rotationToPack the minimum rotation from {@code zUp} to the given {@code vector}.
    *       Modified.
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
    * @param firstVector the first vector. Not modified.
    * @param secondVector the second vector that is rotated with respect to the first vector. Not
    *       modified.
    * @param rotationToPack the minimum rotation from {@code firstVector} to the {@code secondVector}.
    *       Modified.
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
    * {@link EuclidGeometryTools#orientation3DFromFirstToSecondVector3D(double, double, double, double, double, double, Orientation3DBasics)}
    * except that it does not rely on {@code Math#acos(double)} making it faster.
    *
    * @param firstVectorX x-component of the first vector.
    * @param firstVectorY y-component of the first vector.
    * @param firstVectorZ z-component of the first vector.
    * @param secondVectorX x-component of the second vector that is rotated with respect to the first
    *       vector.
    * @param secondVectorY y-component of the second vector that is rotated with respect to the first
    *       vector.
    * @param secondVectorZ z-component of the second vector that is rotated with respect to the first
    *       vector.
    * @param rotationToPack the minimum rotation from {@code firstVector} to the {@code secondVector}.
    *       Modified.
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
    * Given two 2D line segments with finite length, this methods computes two points P &in;
    * lineSegment1 and Q &in; lineSegment2 such that the distance || P - Q || is the minimum distance
    * between the two 2D line segments. <a href="http://geomalgorithms.com/a07-_distance.html"> Useful
    * link</a>.
    *
    * @param lineSegmentStart1 the first endpoint of the first line segment. Not modified.
    * @param lineSegmentEnd1 the second endpoint of the first line segment. Not modified.
    * @param lineSegmentStart2 the first endpoint of the second line segment. Not modified.
    * @param lineSegmentEnd2 the second endpoint of the second line segment. Not modified.
    * @param closestPointOnLineSegment1ToPack the 2D coordinates of the point P are packed in this 2D
    *       point. Modified. Can be {@code null}.
    * @param closestPointOnLineSegment2ToPack the 2D coordinates of the point Q are packed in this 2D
    *       point. Modified. Can be {@code null}.
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
    * @param lineSegmentEnd1 the second endpoint of the first line segment. Not modified.
    * @param lineSegmentStart2 the first endpoint of the second line segment. Not modified.
    * @param lineSegmentEnd2 the second endpoint of the second line segment. Not modified.
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
}