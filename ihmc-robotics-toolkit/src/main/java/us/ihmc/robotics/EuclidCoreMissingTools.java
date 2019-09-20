package us.ihmc.robotics;

import static us.ihmc.euclid.tools.EuclidCoreTools.normSquared;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.CommonMatrix3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
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

      double scale = dotProduct / axis.lengthSquared();
      double projectedX = scale * axis.getX();
      double projectedY = scale * axis.getY();
      double projectedZ = scale * axis.getZ();

      result.set(projectedX, projectedY, projectedZ, rotation.getS());
      result.normalize();
   }

   public static boolean intersectionBetweenTwoLine2Ds(Point2DReadOnly firstPointOnLine1, Point2DReadOnly secondPointOnLine1, Point2DReadOnly firstPointOnLine2,
                                                       Point2DReadOnly secondPointOnLine2, Point2DBasics intersectionToPack)
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
      rotationMatrix3DFromFirstToSecondVector3D(Axis.Z, vector, rotationToPack);
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
   public static void rotationMatrix3DFromFirstToSecondVector3D(Vector3DReadOnly firstVector, Vector3DReadOnly secondVector,
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
   public static void rotationMatrix3DFromFirstToSecondVector3D(double firstVectorX, double firstVectorY, double firstVectorZ, double secondVectorX,
                                                                double secondVectorY, double secondVectorZ, CommonMatrix3DBasics rotationToPack)
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
}
