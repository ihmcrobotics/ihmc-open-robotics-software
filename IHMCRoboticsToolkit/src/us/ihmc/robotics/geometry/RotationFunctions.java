package us.ihmc.robotics.geometry;

import org.apache.commons.math3.exception.util.LocalizedFormats;
import org.apache.commons.math3.geometry.euclidean.threed.NotARotationMatrixException;
import org.apache.commons.math3.util.FastMath;
import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.AxisAngle4f;
import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix3f;
import javax.vecmath.Quat4d;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3d;

public class RotationFunctions
{
   public static final int QUATERNION_SIZE = 4;
   public static final double MATRIX_TO_QUATERNION_THRESHOLD = 1e-10;
   private static final ThreadLocal<Matrix3d> rotationMatrixForQuaternionConvertor = new ThreadLocal<Matrix3d>()
   {
      @Override
      public Matrix3d initialValue()
      {
         return new Matrix3d();
      }
   };

   private static final ThreadLocal<AxisAngle4d> axisAngleForRotationVectorConvertor = new ThreadLocal<AxisAngle4d>()
   {
      @Override
      public AxisAngle4d initialValue()
      {
         return new AxisAngle4d();
      }
   };

   private static final ThreadLocal<RotationVectorToAxisAngleConverter> rotationVectorToAxisAngleConvertor =
      new ThreadLocal<RotationVectorToAxisAngleConverter>()
   {
      @Override
      public RotationVectorToAxisAngleConverter initialValue()
      {
         return new RotationVectorToAxisAngleConverter();
      }
   };

   /**
    * Sets the rotation matrix, based on the yaw, pitch and roll values.
    * @param rotationMatrixToPack the rotation matrix to set, based on the yaw, pitch and roll values
    * @param yaw yaw rotation (about a fixed z-axis)
    * @param pitch pitch rotation (about a fixed y-axis)
    * @param roll roll rotation (about a fixed x-axis)
    */
   public static void setYawPitchRoll(Matrix3d rotationMatrixToPack, double yaw, double pitch, double roll)
   {
      double cAlpha = Math.cos(yaw);
      double sAlpha = Math.sin(yaw);

      double cBeta = Math.cos(pitch);
      double sBeta = Math.sin(pitch);

      double cGamma = Math.cos(roll);
      double sGamma = Math.sin(roll);

      // Introduction to Robotics, 2.64
      rotationMatrixToPack.setElement(0, 0, cAlpha * cBeta);
      rotationMatrixToPack.setElement(0, 1, cAlpha * sBeta * sGamma - sAlpha * cGamma);
      rotationMatrixToPack.setElement(0, 2, cAlpha * sBeta * cGamma + sAlpha * sGamma);

      rotationMatrixToPack.setElement(1, 0, sAlpha * cBeta);
      rotationMatrixToPack.setElement(1, 1, sAlpha * sBeta * sGamma + cAlpha * cGamma);
      rotationMatrixToPack.setElement(1, 2, sAlpha * sBeta * cGamma - cAlpha * sGamma);

      rotationMatrixToPack.setElement(2, 0, -sBeta);
      rotationMatrixToPack.setElement(2, 1, cBeta * sGamma);
      rotationMatrixToPack.setElement(2, 2, cBeta * cGamma);

   }

   public static void setMatrixBasedOnYawPitchAndRoll(Matrix3d rotationMatrixToPack, double[] yawPitchRoll)
   {
      setMatrixBasedOnYawPitchAndRoll(rotationMatrixToPack, yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
   }

   public static void setMatrixBasedOnYawPitchAndRoll(Matrix3d rotationMatrixToPack, double yaw, double pitch, double roll)
   {
      double sina = Math.sin(roll);
      double sinb = Math.sin(pitch);
      double sinc = Math.sin(yaw);
      double cosa = Math.cos(roll);
      double cosb = Math.cos(pitch);
      double cosc = Math.cos(yaw);

      rotationMatrixToPack.m00 = cosb * cosc;
      rotationMatrixToPack.m01 = -(cosa * sinc) + (sina * sinb * cosc);
      rotationMatrixToPack.m02 = (sina * sinc) + (cosa * sinb * cosc);
      rotationMatrixToPack.m10 = cosb * sinc;
      rotationMatrixToPack.m11 = (cosa * cosc) + (sina * sinb * sinc);
      rotationMatrixToPack.m12 = -(sina * cosc) + (cosa * sinb * sinc);
      rotationMatrixToPack.m20 = -sinb;
      rotationMatrixToPack.m21 = sina * cosb;
      rotationMatrixToPack.m22 = cosa * cosb;
   }

   public static double getYaw(Matrix3d rotationMatrix)
   {
      double ret = Math.atan2(rotationMatrix.m10, rotationMatrix.m00);
      if (Double.isNaN(ret))
      {
         throw new RuntimeException("Yaw is NaN! rotationMatrix = " + rotationMatrix);
      }

      return ret;
   }

   public static double getPitch(Matrix3d rotationMatrix)
   {
      double ret = Math.asin(-rotationMatrix.m20);
      if (Double.isNaN(ret))
      {
         throw new RuntimeException("Pitch is NaN! rotationMatrix = " + rotationMatrix);
      }

      return ret;
   }

   public static double getRoll(Matrix3d rotationMatrix)
   {
      double ret = Math.atan2(rotationMatrix.m21, rotationMatrix.m22);
      if (Double.isNaN(ret))
      {
         throw new RuntimeException("Roll is NaN! rotationMatrix = " + rotationMatrix);
      }

      return ret;
   }

   public static void getYawPitchRoll(double[] yawPitchRoll, Matrix3d rotationMatrix)
   {
      yawPitchRoll[0] = getYaw(rotationMatrix);
      yawPitchRoll[1] = getPitch(rotationMatrix);
      yawPitchRoll[2] = getRoll(rotationMatrix);
   }

   public static void getYawPitchRoll(double[] yawPitchRoll, RigidBodyTransform transform)
   {
      Matrix3d rotationMatrix = new Matrix3d();    // TODO: optimize if possible
      transform.getRotation(rotationMatrix);
      getYawPitchRoll(yawPitchRoll, rotationMatrix);
   }

   /**
    * Sets Euler parameters (unit quaternion) based on the given yaw, pitch and roll values.
    * @param q Quat4d to set
    * @param yaw yaw rotation (about a fixed z-axis)
    * @param pitch pitch rotation (about a fixed y-axis)
    * @param roll roll rotation (about a fixed x-axis)
    */
   public static void setQuaternionBasedOnYawPitchRoll(Quat4d q, double yaw, double pitch, double roll)
   {
      double cYaw = Math.cos(yaw / 2.0);
      double sYaw = Math.sin(yaw / 2.0);

      double cPitch = Math.cos(pitch / 2.0);
      double sPitch = Math.sin(pitch / 2.0);

      double cRoll = Math.cos(roll / 2.0);
      double sRoll = Math.sin(roll / 2.0);

      q.w = sPitch * sRoll * sYaw + cPitch * cRoll * cYaw;
      q.x = cPitch * cYaw * sRoll - cRoll * sPitch * sYaw;
      q.y = cPitch * sRoll * sYaw + cRoll * cYaw * sPitch;
      q.z = cPitch * cRoll * sYaw - cYaw * sPitch * sRoll;
   }

   public static void setQuaternionBasedOnYawPitchRoll(Quat4d q, double[] yawPitchRoll)
   {
      setQuaternionBasedOnYawPitchRoll(q, yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
   }

   public static void setYawPitchRollBasedOnQuaternion(double[] yawPitchRollToPack, Quat4d q)
   {
      setYawPitchRollBasedOnQuaternion(yawPitchRollToPack, q.x, q.y, q.z, q.w);
   }
   
   public static void setYawPitchRollBasedOnQuaternion(double[] yawPitchRollToPack, Quat4f q)
   {
      setYawPitchRollBasedOnQuaternion(yawPitchRollToPack, q.x, q.y, q.z, q.w);
   }
   
   
   public static void setYawPitchRollBasedOnQuaternion(double[] yawPitchRollToPack, double x, double y, double z, double w)
   {
      double qxqx = x * x;
      double qyqy = y * y;
      double qzqz = z * z;

      double pitchArgument = -2.0 * x * z + 2.0 * w * y;

      yawPitchRollToPack[1] = Math.asin(pitchArgument);

      if (Math.abs(yawPitchRollToPack[1]) < 0.49 * Math.PI)
      {
         yawPitchRollToPack[0] = Math.atan2(2.0 * x * y + 2.0 * z * w, 1.0 - 2.0 * qyqy - 2.0 * qzqz);    // Math.asin(q_qs.val * q_qz.val * 2.0);
         yawPitchRollToPack[2] = Math.atan2(2.0 * y * z + 2.0 * x * w, 1.0 - 2.0 * qxqx - 2.0 * qyqy);    // Math.asin(q_qs.val * q_qx.val * 2.0);
      }
      else
      {
         yawPitchRollToPack[0] = 2.0 * Math.atan2(z, w);
         yawPitchRollToPack[2] = 0.0;
      }
   }
   
   
   public static void getQuaternionFromYawAndZNormal(double yaw, Vector3d zNormal, Quat4d quaternionToPack)
   {
      double Cx = 1.0;
      double Cy = Math.tan(yaw);
      if (Math.abs(zNormal.z) < 1e-9)
      {
         quaternionToPack.set(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
         return;
      }
      double Cz = -1.0*(zNormal.x + Cy*zNormal.y) / zNormal.z;
      double CT = Math.sqrt(Cx*Cx + Cy*Cy + Cz*Cz);
      if (CT < 1e-9) throw new RuntimeException("Error calculating Quaternion");

      Vector3d xAxis = new Vector3d(Cx/CT, Cy/CT, Cz/CT);
      if (xAxis.x * Math.cos(yaw) + xAxis.y * Math.sin(yaw) < 0.0)
      {
         xAxis.negate();
      }
      Vector3d yAxis = new Vector3d();
      Vector3d zAxis = zNormal;
      yAxis.cross(zAxis, xAxis);

      Matrix3d rotationMatrix = new Matrix3d(xAxis.x, yAxis.x, zAxis.x, xAxis.y, yAxis.y, zAxis.y, xAxis.z, yAxis.z, zAxis.z);
      try
      {
         RotationFunctions.setQuaternionBasedOnMatrix3d(quaternionToPack, rotationMatrix);
      }
      catch(Exception e)
      {
         System.err.println("Trouble with getQuaternionFromYawAndZNormal. yaw = " + yaw + ", zNormal = " + zNormal);
      }

   }
   
   public static double getYawFromQuaternion(Quat4d q){
	   double[] ypr = new double[3];
	   setYawPitchRollBasedOnQuaternion(ypr, q);
	   return ypr[0];
   }
   
   public static double getPitchFromQuaternion(Quat4d rotationToPack)
   {
      double[] ypr = new double[3];
      setYawPitchRollBasedOnQuaternion(ypr, rotationToPack);
      return ypr[1];
   }
   
   public static void quaternionToMatrix(DenseMatrix64F matrix, Quat4d quaternion, int rowStart)
   {
      int index = rowStart;
      matrix.set(index++, 0, quaternion.getX());
      matrix.set(index++, 0, quaternion.getY());
      matrix.set(index++, 0, quaternion.getZ());
      matrix.set(index++, 0, quaternion.getW());
   }

   public static void matrixToQuaternion(Quat4d quaternion, DenseMatrix64F matrix, int rowStart)
   {
      int index = rowStart;
      double x = matrix.get(index++, 0);
      double y = matrix.get(index++, 0);
      double z = matrix.get(index++, 0);
      double w = matrix.get(index++, 0);
      quaternion.set(x, y, z, w);
   }


   public static void setAxisAngleBasedOnRotationVector(AxisAngle4d axisAngleToPack, Vector3d rotationVector)
   {
      rotationVectorToAxisAngleConvertor.get().convertRotationVectorToAxisAngle(axisAngleToPack, rotationVector);
   }

   public static void setMatrixBasedOnRotationVector(Matrix3d rotationMatrix, Vector3d rotationVector)
   {
      AxisAngle4d localAxisAngle = axisAngleForRotationVectorConvertor.get();
      setAxisAngleBasedOnRotationVector(localAxisAngle, rotationVector);
      rotationMatrix.set(localAxisAngle);
   }

   public static void assertQuaternionNormalized(Quat4d q, String errorMessage)
   {
      double normSquaredQuaternion = (q.w * q.w) + (q.x * q.x) + (q.y * q.y) + (q.z * q.z);
      if (!MathTools.isFinite(normSquaredQuaternion))
         throw new AssertionError("quaternion has non-finite norm \n quaternion is " + q);
      if (!MathTools.epsilonEquals(normSquaredQuaternion, 1, 1e-3))
         throw new AssertionError("quaternion is not normalized. quaternion is " + q + " with normalization error " + Math.sqrt(normSquaredQuaternion));
   }

   public static boolean isQuaternionNormalized(Quat4d q)
   {
      double normSquaredQuaternion = (q.w * q.w) + (q.x * q.x) + (q.y * q.y) + (q.z * q.z);
      if (!MathTools.isFinite(normSquaredQuaternion))
         return false;
      if (!MathTools.epsilonEquals(normSquaredQuaternion, 1, 1e-3))
         return false;

      return true;
   }

   public static void assertProper(Matrix3d matrix3d)
   {
//      if((RotationFunctions.isRotationProper(matrix3d)))
//      {
//         throw new RuntimeException("Rotation is not proper.");
//      }
      assert RotationFunctions.isRotationProper(matrix3d) : "Matrix3d: " + matrix3d + " is not proper";
   }

   public static boolean isRotationProper(Matrix3d mat)
   {
      boolean determinantZero = Math.abs(mat.determinant() - 1.0) < 1e-10;

      Matrix3d orthogonalityCheck = new Matrix3d();
      orthogonalityCheck.mulTransposeRight(mat, mat);

      boolean orthogonal = MatrixTools.isIdentity(orthogonalityCheck, 1e-10);
      boolean ret = orthogonal && determinantZero;

      // if (!ret)
      // throw new RuntimeException();
      return ret;
   }

   public static void checkUnitLength(Quat4d orientation)
   {
      double lengthSquared = orientation.w * orientation.w + orientation.x * orientation.x + orientation.y * orientation.y + orientation.z * orientation.z;
      MathTools.checkIfEqual(lengthSquared, 1.0, 1e-5);
   }

   public static boolean isNaNorInf(RigidBodyTransform transformIn)
   {
      double[] transform = new double[16];
      transformIn.get(transform);

      for (double e : transform)
      {
         if (Double.isInfinite(e) || Double.isNaN(e))
         {
            return true;
         }
      }

      return false;
   }

   public static boolean quaternionEpsilonEquals(Quat4d original, Quat4d result, double epsilon)
   {
      if (original.epsilonEquals(result, epsilon))
      {
         return true;
      }
      else
      {
         Quat4d temp = new Quat4d();
         temp.negate(original);

         return temp.epsilonEquals(result, epsilon);
      }
   }
   public static boolean quaternionEpsilonEquals(Quat4f original, Quat4f result, float epsilon)
   {
      return quaternionEpsilonEquals(new Quat4d(original), new Quat4d(result), epsilon);
   }

   public enum ComparisonMode
   {
      IGNORE_FLIPPED_AXES, IGNORE_FLIPPED_AXES_ROTATION_DIRECTION_AND_COMPLETE_ROTATIONS;
   }
   
   public static boolean axisAngleEpsilonEquivalent(AxisAngle4d original, AxisAngle4d result, double epsilon, ComparisonMode mode)
   {
      if (mode == ComparisonMode.IGNORE_FLIPPED_AXES)
      {
         return axisAngleEpsilonEqualsIgnoreFlippedAxes(original, result, epsilon);
      }
      else if (mode == ComparisonMode.IGNORE_FLIPPED_AXES_ROTATION_DIRECTION_AND_COMPLETE_ROTATIONS)
      {
         AxisAngle4d originalMinusPiToPi = getAxisAngleMinusPiToPiCopy(original);
         AxisAngle4d resultMinusPiToPi = getAxisAngleMinusPiToPiCopy(result);

         boolean originalAxisAngleIsZero = MathTools.epsilonEquals(originalMinusPiToPi.angle, 0.0, 0.1 * epsilon);
         boolean resultAxisAngleIsZero = MathTools.epsilonEquals(resultMinusPiToPi.angle, 0.0, 0.1 * epsilon);
         
         boolean originalAngleIs180 = MathTools.epsilonEquals(Math.abs(originalMinusPiToPi.angle), Math.PI, 0.1 * epsilon);
         boolean resultAngleIs180 = MathTools.epsilonEquals(Math.abs(resultMinusPiToPi.angle), Math.PI, 0.1 * epsilon);

         if (originalAxisAngleIsZero && resultAxisAngleIsZero)
         {
            return true;
         }
         else if (originalAngleIs180 && resultAngleIs180)
         {
            return axisAngleEpsilonEqualsIgnoreFlippedAxes(original, result, epsilon);
         }
         else
         {
            return axisAngleEpsilonEqualsIgnoreFlippedAxes(originalMinusPiToPi, resultMinusPiToPi, epsilon);
         }
      }
      else
      {
         return original.epsilonEquals(result, epsilon);
      }
   }
   
   private static AxisAngle4d getAxisAngleMinusPiToPiCopy(AxisAngle4d axisAngle4d)
   {
      AxisAngle4d ret = new AxisAngle4d(axisAngle4d);
      ret.angle = AngleTools.trimAngleMinusPiToPi(ret.angle);
      return ret;
   }
   
   public static boolean axisAngleEpsilonEqualsIgnoreFlippedAxes(AxisAngle4d original, AxisAngle4d result, double epsilon)
   {
      if (original.epsilonEquals(result, epsilon))
      {
         return true;
      }
      else
      {
         AxisAngle4d temp = new AxisAngle4d(original);
         temp.angle *= -1.0;
         temp.x *= -1.0;
         temp.y *= -1.0;
         temp.z *= -1.0;

         boolean axisAnglesEqualWhenAxisAndAngleAreFlipped = temp.epsilonEquals(result, epsilon);
         
         return axisAnglesEqualWhenAxisAndAngleAreFlipped;
      }
   }

   //utility function workaround for 
   // https://www.mail-archive.com/java3d-interest@java.sun.com/msg09568.html
   private static ThreadLocal<Quat4d> threadLocalTemporaryQuaternion = new ThreadLocal<Quat4d>()
   {
      @Override
      protected Quat4d initialValue()
      {
         return new Quat4d();
      }
   };
   
   public static void axisAngleFromMatrix(Matrix3f matrix, AxisAngle4f axisAngle4fToPack)
   {
      threadLocalTemporaryQuaternion.get().set(matrix);
      axisAngle4fToPack.set(threadLocalTemporaryQuaternion.get());
   }
   
   public static void axisAngleFromMatrix(Matrix3d matrix, AxisAngle4d axisAngle4dToPack)
   {
      threadLocalTemporaryQuaternion.get().set(matrix);
      axisAngle4dToPack.set(threadLocalTemporaryQuaternion.get());
      
   }
   
   public static void setQuaternionBasedOnTransform(Quat4d quaternionToPack, RigidBodyTransform transform)
   {
      Matrix3d rotationMatrix = rotationMatrixForQuaternionConvertor.get();
      transform.getRotation(rotationMatrix);
      setQuaternionBasedOnMatrix3d(quaternionToPack, rotationMatrix);
   }

   private static ThreadLocal<Matrix3d> tempMatrix3ds = new ThreadLocal<Matrix3d>()
   {
      @Override
      protected Matrix3d initialValue()
      {
         return new Matrix3d();
      }
   };

   /**
    * From Apache... Convert an orthogonal rotation matrix to a quaternion.
    * @param rotationMatrix orthogonal rotation matrix
    * @return quaternion corresponding to the matrix
    */
   public static void setQuaternionBasedOnMatrix3d(Quat4d quaternionToPack, Matrix3d rotationMatrix)
   {
      Matrix3d orthogonalizedMatrix = tempMatrix3ds.get();

      orthogonalizeMatrix(orthogonalizedMatrix, rotationMatrix, MATRIX_TO_QUATERNION_THRESHOLD);
      rotationMatrix = orthogonalizedMatrix;

      // There are different ways to compute the quaternions elements
      // from the matrix. They all involve computing one element from
      // the diagonal of the matrix, and computing the three other ones
      // using a formula involving a division by the first element,
      // which unfortunately can be zero. Since the norm of the
      // quaternion is 1, we know at least one element has an absolute
      // value greater or equal to 0.5, so it is always possible to
      // select the right formula and avoid division by zero and even
      // numerical inaccuracy. Checking the elements in turn and using
      // the first one greater than 0.45 is safe (this leads to a simple
      // test since qi = 0.45 implies 4 qi^2 - 1 = -0.19)
      double s = rotationMatrix.getElement(0, 0) + rotationMatrix.getElement(1, 1) + rotationMatrix.getElement(2, 2);
      if (s > -0.19)
      {
         // compute q0 and deduce q1, q2 and q3
         quaternionToPack.w = 0.5 * FastMath.sqrt(s + 1.0);
         double inv = 0.25 / quaternionToPack.w;
         quaternionToPack.x = inv * (rotationMatrix.getElement(2, 1) - rotationMatrix.getElement(1, 2));
         quaternionToPack.y = inv * (rotationMatrix.getElement(0, 2) - rotationMatrix.getElement(2, 0));
         quaternionToPack.z = inv * (rotationMatrix.getElement(1, 0) - rotationMatrix.getElement(0, 1));
      }
      else
      {
         s = rotationMatrix.getElement(0, 0) - rotationMatrix.getElement(1, 1) - rotationMatrix.getElement(2, 2);

         if (s > -0.19)
         {
            // compute q1 and deduce q0, q2 and q3
            quaternionToPack.x = 0.5 * FastMath.sqrt(s + 1.0);
            double inv = 0.25 / quaternionToPack.x;
            quaternionToPack.w = inv * (rotationMatrix.getElement(2, 1) - rotationMatrix.getElement(1, 2));
            quaternionToPack.y = inv * (rotationMatrix.getElement(1, 0) + rotationMatrix.getElement(0, 1));
            quaternionToPack.z = inv * (rotationMatrix.getElement(2, 0) + rotationMatrix.getElement(0, 2));
         }
         else
         {
            s = rotationMatrix.getElement(1, 1) - rotationMatrix.getElement(0, 0) - rotationMatrix.getElement(2, 2);

            if (s > -0.19)
            {
               // compute q2 and deduce q0, q1 and q3
               quaternionToPack.y = 0.5 * FastMath.sqrt(s + 1.0);
               double inv = 0.25 / quaternionToPack.y;
               quaternionToPack.w = inv * (rotationMatrix.getElement(0, 2) - rotationMatrix.getElement(2, 0));
               quaternionToPack.x = inv * (rotationMatrix.getElement(1, 0) + rotationMatrix.getElement(0, 1));
               quaternionToPack.z = inv * (rotationMatrix.getElement(1, 2) + rotationMatrix.getElement(2, 1));
            }
            else
            {
               // compute q3 and deduce q0, q1 and q2
               s = rotationMatrix.getElement(2, 2) - rotationMatrix.getElement(0, 0) - rotationMatrix.getElement(1, 1);
               quaternionToPack.z = 0.5 * FastMath.sqrt(s + 1.0);
               double inv = 0.25 / quaternionToPack.z;
               quaternionToPack.w = inv * (rotationMatrix.getElement(1, 0) - rotationMatrix.getElement(0, 1));
               quaternionToPack.x = inv * (rotationMatrix.getElement(2, 0) + rotationMatrix.getElement(0, 2));
               quaternionToPack.y = inv * (rotationMatrix.getElement(1, 2) + rotationMatrix.getElement(2, 1));
            }
         }
      }

//    quaternionToPack.normalize();
   }

   public static void setQuaternionBasedOnMatrix3d(Quat4f quaternionToPack, Matrix3d rotationMatrix)
   {
      Matrix3d orthogonalizedMatrix = tempMatrix3ds.get();

      orthogonalizeMatrix(orthogonalizedMatrix, rotationMatrix, MATRIX_TO_QUATERNION_THRESHOLD);
      rotationMatrix = orthogonalizedMatrix;

      // There are different ways to compute the quaternions elements
      // from the matrix. They all involve computing one element from
      // the diagonal of the matrix, and computing the three other ones
      // using a formula involving a division by the first element,
      // which unfortunately can be zero. Since the norm of the
      // quaternion is 1, we know at least one element has an absolute
      // value greater or equal to 0.5, so it is always possible to
      // select the right formula and avoid division by zero and even
      // numerical inaccuracy. Checking the elements in turn and using
      // the first one greater than 0.45 is safe (this leads to a simple
      // test since qi = 0.45 implies 4 qi^2 - 1 = -0.19)
      double s = rotationMatrix.getElement(0, 0) + rotationMatrix.getElement(1, 1) + rotationMatrix.getElement(2, 2);
      if (s > -0.19)
      {
         // compute q0 and deduce q1, q2 and q3
         quaternionToPack.w = (float) (0.5 * FastMath.sqrt(s + 1.0));
         double inv = 0.25 / quaternionToPack.w;
         quaternionToPack.x = (float) (inv * (rotationMatrix.getElement(2, 1) - rotationMatrix.getElement(1, 2)));
         quaternionToPack.y = (float) (inv * (rotationMatrix.getElement(0, 2) - rotationMatrix.getElement(2, 0)));
         quaternionToPack.z = (float) (inv * (rotationMatrix.getElement(1, 0) - rotationMatrix.getElement(0, 1)));
      }
      else
      {
         s = rotationMatrix.getElement(0, 0) - rotationMatrix.getElement(1, 1) - rotationMatrix.getElement(2, 2);

         if (s > -0.19)
         {
            // compute q1 and deduce q0, q2 and q3
            quaternionToPack.x = (float) (0.5 * FastMath.sqrt(s + 1.0));
            double inv = 0.25 / quaternionToPack.x;
            quaternionToPack.w = (float) (inv * (rotationMatrix.getElement(2, 1) - rotationMatrix.getElement(1, 2)));
            quaternionToPack.y = (float) (inv * (rotationMatrix.getElement(1, 0) + rotationMatrix.getElement(0, 1)));
            quaternionToPack.z = (float) (inv * (rotationMatrix.getElement(2, 0) + rotationMatrix.getElement(0, 2)));
         }
         else
         {
            s = rotationMatrix.getElement(1, 1) - rotationMatrix.getElement(0, 0) - rotationMatrix.getElement(2, 2);

            if (s > -0.19)
            {
               // compute q2 and deduce q0, q1 and q3
               quaternionToPack.y = (float) (0.5 * FastMath.sqrt(s + 1.0));
               double inv = 0.25 / quaternionToPack.y;
               quaternionToPack.w = (float) (inv * (rotationMatrix.getElement(0, 2) - rotationMatrix.getElement(2, 0)));
               quaternionToPack.x = (float) (inv * (rotationMatrix.getElement(1, 0) + rotationMatrix.getElement(0, 1)));
               quaternionToPack.z = (float) (inv * (rotationMatrix.getElement(1, 2) + rotationMatrix.getElement(2, 1)));
            }
            else
            {
               // compute q3 and deduce q0, q1 and q2
               s = rotationMatrix.getElement(2, 2) - rotationMatrix.getElement(0, 0) - rotationMatrix.getElement(1, 1);
               quaternionToPack.z = (float) (0.5 * FastMath.sqrt(s + 1.0));
               double inv = 0.25 / quaternionToPack.z;
               quaternionToPack.w = (float) (inv * (rotationMatrix.getElement(1, 0) - rotationMatrix.getElement(0, 1)));
               quaternionToPack.x = (float) (inv * (rotationMatrix.getElement(2, 0) + rotationMatrix.getElement(0, 2)));
               quaternionToPack.y = (float) (inv * (rotationMatrix.getElement(1, 2) + rotationMatrix.getElement(2, 1)));
            }
         }
      }

      //    quaternionToPack.normalize();
   }


   private static ThreadLocal<Matrix3d> tempXMatrix3d = new ThreadLocal<Matrix3d>()
   {
      @Override
      protected Matrix3d initialValue()
      {
         return new Matrix3d();
      }
   };

   private static ThreadLocal<Matrix3d> tempMXMatrix3d = new ThreadLocal<Matrix3d>()
   {
      @Override
      protected Matrix3d initialValue()
      {
         return new Matrix3d();
      }
   };

   private static ThreadLocal<Matrix3d> tempCorrMatrix3d = new ThreadLocal<Matrix3d>()
   {
      @Override
      protected Matrix3d initialValue()
      {
         return new Matrix3d();
      }
   };

   
   /**
    * From Apache. Perfect orthogonality on a 3X3 matrix.
    * @param matrixToOrthogonalize initial matrix (not exactly orthogonal)
    * @param threshold convergence threshold for the iterative
    * orthogonality correction (convergence is reached when the
    * difference between two steps of the Frobenius norm of the
    * correction is below this threshold)
    * @return an orthogonal matrix close to m
    * @exception NotARotationMatrixException if the matrix cannot be
    * orthogonalized with the given threshold after 10 iterations
    */
   public static void orthogonalizeMatrix(Matrix3d matrixResultToPack, Matrix3d matrixToOrthogonalize, double threshold) throws NotARotationMatrixException
   {
      Matrix3d x = tempXMatrix3d.get();
      Matrix3d mx = tempMXMatrix3d.get();
      Matrix3d corr = tempCorrMatrix3d.get();
      
      x.transpose(matrixToOrthogonalize);
      
      double fn = 0;
      double fn1;

      // iterative correction: Xn+1 = Xn - 0.5 * (Xn.Mt.Xn - M)
      int i = 0;
      while (++i < 11)
      {
         // Mt.Xn
         mx.m00 = x.m00 * x.m00 + x.m10 * x.m10 + x.m20 * x.m20;
         mx.m10 = x.m01 * x.m00 + x.m11 * x.m10 + x.m21 * x.m20;
         mx.m20 = x.m02 * x.m00 + x.m12 * x.m10 + x.m22 * x.m20;
         mx.m01 = x.m00 * x.m01 + x.m10 * x.m11 + x.m20 * x.m21;
         mx.m11 = x.m01 * x.m01 + x.m11 * x.m11 + x.m21 * x.m21;
         mx.m21 = x.m02 * x.m01 + x.m12 * x.m11 + x.m22 * x.m21;
         mx.m02 = x.m00 * x.m02 + x.m10 * x.m12 + x.m20 * x.m22;
         mx.m12 = x.m01 * x.m02 + x.m11 * x.m12 + x.m21 * x.m22;
         mx.m22 = x.m02 * x.m02 + x.m12 * x.m12 + x.m22 * x.m22;

         // Xn+1
         matrixResultToPack.setElement(0, 0, x.m00 - 0.5 * (x.m00 * mx.m00 + x.m01 * mx.m10 + x.m02 * mx.m20 - x.m00));
         matrixResultToPack.setElement(1, 0, x.m01 - 0.5 * (x.m00 * mx.m01 + x.m01 * mx.m11 + x.m02 * mx.m21 - x.m01));
         matrixResultToPack.setElement(2, 0, x.m02 - 0.5 * (x.m00 * mx.m02 + x.m01 * mx.m12 + x.m02 * mx.m22 - x.m02));
         matrixResultToPack.setElement(0, 1, x.m10 - 0.5 * (x.m10 * mx.m00 + x.m11 * mx.m10 + x.m12 * mx.m20 - x.m10));
         matrixResultToPack.setElement(1, 1, x.m11 - 0.5 * (x.m10 * mx.m01 + x.m11 * mx.m11 + x.m12 * mx.m21 - x.m11));
         matrixResultToPack.setElement(2, 1, x.m12 - 0.5 * (x.m10 * mx.m02 + x.m11 * mx.m12 + x.m12 * mx.m22 - x.m12));
         matrixResultToPack.setElement(0, 2, x.m20 - 0.5 * (x.m20 * mx.m00 + x.m21 * mx.m10 + x.m22 * mx.m20 - x.m20));
         matrixResultToPack.setElement(1, 2, x.m21 - 0.5 * (x.m20 * mx.m01 + x.m21 * mx.m11 + x.m22 * mx.m21 - x.m21));
         matrixResultToPack.setElement(2, 2, x.m22 - 0.5 * (x.m20 * mx.m02 + x.m21 * mx.m12 + x.m22 * mx.m22 - x.m22));

         // correction on each elements
         corr.m00 = matrixResultToPack.getElement(0, 0) - x.m00;
         corr.m01 = matrixResultToPack.getElement(1, 0) - x.m01;
         corr.m02 = matrixResultToPack.getElement(2, 0) - x.m02;
         corr.m10 = matrixResultToPack.getElement(0, 1) - x.m10;
         corr.m11 = matrixResultToPack.getElement(1, 1) - x.m11;
         corr.m12 = matrixResultToPack.getElement(2, 1) - x.m12;
         corr.m20 = matrixResultToPack.getElement(0, 2) - x.m20;
         corr.m21 = matrixResultToPack.getElement(1, 2) - x.m21;
         corr.m22 = matrixResultToPack.getElement(2, 2) - x.m22;

         // Frobenius norm of the correction
         fn1 = corr.m00 * corr.m00 + corr.m01 * corr.m01 + corr.m02 * corr.m02 + corr.m10 * corr.m10 + corr.m11 * corr.m11 + corr.m12 * corr.m12 + corr.m20 * corr.m20 + corr.m21 * corr.m21
               + corr.m22 * corr.m22;

         // convergence test
         if (FastMath.abs(fn1 - fn) <= threshold)
         {
            return;
         }

         // prepare next iteration
         x.m00 = matrixResultToPack.getElement(0, 0);
         x.m01 = matrixResultToPack.getElement(1, 0);
         x.m02 = matrixResultToPack.getElement(2, 0);
         x.m10 = matrixResultToPack.getElement(0, 1);
         x.m11 = matrixResultToPack.getElement(1, 1);
         x.m12 = matrixResultToPack.getElement(2, 1);
         x.m20 = matrixResultToPack.getElement(0, 2);
         x.m21 = matrixResultToPack.getElement(1, 2);
         x.m22 = matrixResultToPack.getElement(2, 2);
         fn = fn1;
      }

      // the algorithm did not converge after 10 iterations
      throw new NotARotationMatrixException(LocalizedFormats.UNABLE_TO_ORTHOGONOLIZE_MATRIX, i - 1);
   }

   
}
