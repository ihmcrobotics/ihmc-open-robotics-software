package us.ihmc.euclid.tools;

import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.*;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class EuclidCoreMissingTools
{
   public static boolean isFinite(Tuple3DBasics tuple)
   {
      return Double.isFinite(tuple.getX()) && Double.isFinite(tuple.getY()) && Double.isFinite(tuple.getZ());
   }

   public static boolean epsilonEquals(RigidBodyTransformReadOnly a, RigidBodyTransformReadOnly b, double rotationEpsilon, double translationEpsilon)
   {
      return a.getRotation().geometricallyEquals(b.getRotation(), rotationEpsilon)
          && a.getTranslation().geometricallyEquals(b.getTranslation(), translationEpsilon);
   }

   public static double angleFromFirstToSecondVector3D(double firstVectorX,
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

   /**
    * Sets the yaw pitch roll but the doubles are given in degrees.
    */
   public static void setYawPitchRollDegrees(Orientation3DBasics orientation3DBasics, double yaw, double pitch, double roll)
   {
      orientation3DBasics.setYawPitchRoll(Math.toRadians(yaw), Math.toRadians(pitch), Math.toRadians(roll));
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
    * @param inputTagentialPartToPack the tuple used to store the tangential part of the input.
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

   public static void transform(Matrix3DReadOnly matrix, double xOriginal, double yOriginal, double zOriginal, Tuple3DBasics tupleTransformed)
   {
      double x = matrix.getM00() * xOriginal + matrix.getM01() * yOriginal + matrix.getM02() * zOriginal;
      double y = matrix.getM10() * xOriginal + matrix.getM11() * yOriginal + matrix.getM12() * zOriginal;
      double z = matrix.getM20() * xOriginal + matrix.getM21() * yOriginal + matrix.getM22() * zOriginal;
      tupleTransformed.set(x, y, z);
   }

   /**
    * Assert on a component basis is the {@code tuple} is equal to (0, 0, 0) given the tolerance
    * {@code epsilon}.
    *
    * @param tuple   the query. Not modified.
    * @param epsilon the tolerance.
    * @return {@code true} if the tuple's component are all equal to zero, {@code false} otherwise.
    */
   public static boolean isZero(Tuple3DReadOnly tuple, double epsilon)
   {
      if (!EuclidCoreTools.epsilonEquals(tuple.getX(), 0.0, epsilon))
         return false;
      if (!EuclidCoreTools.epsilonEquals(tuple.getY(), 0.0, epsilon))
         return false;
      if (!EuclidCoreTools.epsilonEquals(tuple.getZ(), 0.0, epsilon))
         return false;
      return true;
   }

   /**
    * Assert on a component basis is the {@code tuple} is equal to (0, 0) given the tolerance
    * {@code epsilon}.
    *
    * @param tuple   the query. Not modified.
    * @param epsilon the tolerance.
    * @return {@code true} if the tuple's component are all equal to zero, {@code false} otherwise.
    */
   public static boolean isZero(Tuple2DReadOnly tuple, double epsilon)
   {
      if (!EuclidCoreTools.epsilonEquals(tuple.getX(), 0.0, epsilon))
         return false;
      if (!EuclidCoreTools.epsilonEquals(tuple.getY(), 0.0, epsilon))
         return false;
      return true;
   }
}
