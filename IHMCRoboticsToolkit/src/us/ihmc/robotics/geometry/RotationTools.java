package us.ihmc.robotics.geometry;

import static us.ihmc.robotics.MathTools.square;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.AxisAngle4f;
import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix3f;
import javax.vecmath.Quat4d;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3d;

import org.apache.commons.math3.exception.util.LocalizedFormats;
import org.apache.commons.math3.geometry.euclidean.threed.NotARotationMatrixException;
import org.apache.commons.math3.util.FastMath;
import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;

public class RotationTools
{
   public enum AxisAngleComparisonMode
   {
      IGNORE_FLIPPED_AXES, IGNORE_FLIPPED_AXES_ROTATION_DIRECTION_AND_COMPLETE_ROTATIONS;
   }
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

   private static final ThreadLocal<Matrix3d> rotationMatrixForYawPitchRollConvertor = new ThreadLocal<Matrix3d>()
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

   private static final ThreadLocal<RotationVectorToAxisAngleConverter> rotationVectorToAxisAngleConvertor = new ThreadLocal<RotationVectorToAxisAngleConverter>()
   {
      @Override
      public RotationVectorToAxisAngleConverter initialValue()
      {
         return new RotationVectorToAxisAngleConverter();
      }
   };

   private static final ThreadLocal<Matrix3d> rotationMatrixForIsRotationProper = new ThreadLocal<Matrix3d>()
   {
      @Override
      public Matrix3d initialValue()
      {
         return new Matrix3d();
      }
   };

   private static final ThreadLocal<double[]> yawPitchRollForQuaternionToYawPitchRollConvertor = new ThreadLocal<double[]>()
   {
      @Override
      public double[] initialValue()
      {
         return new double[3];
      }
   };

   private static final ThreadLocal<Quat4d> quaternionForEpsilonEquals = new ThreadLocal<Quat4d>()
   {
      @Override
      public Quat4d initialValue()
      {
         return new Quat4d();
      }
   };

   private static final ThreadLocal<Quat4f> quaternionFloatForEpsilonEquals = new ThreadLocal<Quat4f>()
   {
      @Override
      public Quat4f initialValue()
      {
         return new Quat4f();
      }
   };

   private static final ThreadLocal<AxisAngle4d> originalAxisAngleForEpsilonEquals = new ThreadLocal<AxisAngle4d>()
   {
      @Override
      public AxisAngle4d initialValue()
      {
         return new AxisAngle4d();
      }
   };

   private static final ThreadLocal<AxisAngle4d> resultAxisAngleForEpsilonEquals = new ThreadLocal<AxisAngle4d>()
   {
      @Override
      public AxisAngle4d initialValue()
      {
         return new AxisAngle4d();
      }
   };

   private static final ThreadLocal<Quat4d> quaternionForMatrixToQuaternionFloatConvertor = new ThreadLocal<Quat4d>()
   {
      @Override
      public Quat4d initialValue()
      {
         return new Quat4d();
      }
   };

   private static ThreadLocal<Quat4d> threadLocalTemporaryQuaternion = new ThreadLocal<Quat4d>()
   {
      @Override
      protected Quat4d initialValue()
      {
         return new Quat4d();
      }
   };

   private static ThreadLocal<Matrix3d> rotationMatrixForQuaternionFromYawAndZNormal = new ThreadLocal<Matrix3d>()
   {
      @Override
      protected Matrix3d initialValue()
      {
         return new Matrix3d();
      }
   };

   private static ThreadLocal<Matrix3d> tempMatrix3ds = new ThreadLocal<Matrix3d>()
   {
      @Override
      protected Matrix3d initialValue()
      {
         return new Matrix3d();
      }
   };

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

   private static ThreadLocal<Matrix3d> rotationMatrixForConvertingFloatToDouble = new ThreadLocal<Matrix3d>()
   {
      @Override
      protected Matrix3d initialValue()
      {
         return new Matrix3d();
      }
   };

   /**
    * Compute the yaw rotation from the yaw-pitch-roll rotations induced by the given rotationMatrix.
    * @param rotationMatrix rotation matrix from which the yaw is computed.
    * @return yaw rotation (around the z-axis).
    */
   public static double computeYaw(Matrix3d rotationMatrix)
   {
      double yaw = Math.atan2(rotationMatrix.m10, rotationMatrix.m00);
      if (Double.isNaN(yaw))
      {
         throw new RuntimeException("Yaw is NaN! rotationMatrix = " + rotationMatrix);
      }

      return yaw;
   }

   /**
    * Compute the yaw rotation from the yaw-pitch-roll rotations induced by the given quaternion.
    * 
    * <p> The three rotations yaw-pitch-roll need to be computed to get the yaw. If you need more than one of the three rotations, prefer using {@link #convertQuaternionToYawPitchRoll(Quat4d, double[])}. </p>
    * 
    * @param quaternion unit quaternion from which the yaw is computed.
    * @return yaw rotation (around the z-axis).
    */
   public static double computeYaw(Quat4d quaternion)
   {
      double[] yawPitchRoll = yawPitchRollForQuaternionToYawPitchRollConvertor.get();
      convertQuaternionToYawPitchRoll(quaternion, yawPitchRoll);
      return yawPitchRoll[0];
   }

   /**
    * Compute the pitch rotation from the yaw-pitch-roll rotations induced by the given rotationMatrix.
    * @param rotationMatrix rotation matrix from which the pitch is computed.
    * @return pitch rotation (around the y-axis).
    */
   public static double computePitch(Matrix3d rotationMatrix)
   {
      double pitch = Math.asin(-rotationMatrix.m20);
      if (Double.isNaN(pitch))
      {
         throw new RuntimeException("Pitch is NaN! rotationMatrix = " + rotationMatrix);
      }

      return pitch;
   }

   /**
    * Compute the pitch rotation from the yaw-pitch-roll rotations induced by the given quaternion.
    * 
    * <p> The three rotations yaw-pitch-roll need to be computed to get the pitch. If you need more than one of the three rotations, prefer using {@link #convertQuaternionToYawPitchRoll(Quat4d, double[])}. </p>
    * 
    * @param quaternion unit quaternion from which the pitch is computed.
    * @return pitch rotation (around the y-axis).
    */
   public static double computePitch(Quat4d quaternion)
   {
      double[] yawPitchRoll = yawPitchRollForQuaternionToYawPitchRollConvertor.get();
      convertQuaternionToYawPitchRoll(quaternion, yawPitchRoll);
      return yawPitchRoll[1];
   }

   /**
    * Compute the roll rotation from the yaw-pitch-roll rotations induced by the given rotationMatrix.
    * @param rotationMatrix rotation matrix from which the roll is computed.
    * @return roll rotation (around the x-axis).
    */
   public static double computeRoll(Matrix3d rotationMatrix)
   {
      double roll = Math.atan2(rotationMatrix.m21, rotationMatrix.m22);
      if (Double.isNaN(roll))
      {
         throw new RuntimeException("Roll is NaN! rotationMatrix = " + rotationMatrix);
      }

      return roll;
   }

   /**
    * Compute the roll rotation from the yaw-pitch-roll rotations induced by the given quaternion.
    * 
    * <p> The three rotations yaw-pitch-roll need to be computed to get the roll. If you need more than one of the three rotations, prefer using {@link #convertQuaternionToYawPitchRoll(Quat4d, double[])}. </p>
    * 
    * @param quaternion unit quaternion from which the roll is computed.
    * @return roll rotation (around the x-axis).
    */
   public static double computeRoll(Quat4d quaternion)
   {
      double[] yawPitchRoll = yawPitchRollForQuaternionToYawPitchRollConvertor.get();
      convertQuaternionToYawPitchRoll(quaternion, yawPitchRoll);
      return yawPitchRoll[2];
   }

   public static void computeQuaternionFromYawAndZNormal(double yaw, Vector3d zNormal, Quat4d quaternionToPack)
   {
      double Cx = 1.0;
      double Cy = Math.tan(yaw);
      if (Math.abs(zNormal.z) < 1e-9)
      {
         quaternionToPack.set(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
         return;
      }
      double Cz = -1.0 * (zNormal.x + Cy * zNormal.y) / zNormal.z;
      double CT = Math.sqrt(Cx * Cx + Cy * Cy + Cz * Cz);
      if (CT < 1e-9)
         throw new RuntimeException("Error calculating Quaternion");

      Vector3d xAxis = new Vector3d(Cx / CT, Cy / CT, Cz / CT);
      if (xAxis.x * Math.cos(yaw) + xAxis.y * Math.sin(yaw) < 0.0)
      {
         xAxis.negate();
      }
      Vector3d yAxis = new Vector3d();
      Vector3d zAxis = zNormal;
      yAxis.cross(zAxis, xAxis);

      Matrix3d rotationMatrix = rotationMatrixForQuaternionFromYawAndZNormal.get();
      rotationMatrix.setColumn(0, xAxis);
      rotationMatrix.setColumn(1, yAxis);
      rotationMatrix.setColumn(2, zAxis);

      try
      {
         convertMatrixToQuaternion(rotationMatrix, quaternionToPack);
      }
      catch (Exception e)
      {
         System.err.println("Trouble with getQuaternionFromYawAndZNormal. yaw = " + yaw + ", zNormal = " + zNormal);
      }
   }

   //utility function workaround for
   // https://www.mail-archive.com/java3d-interest@java.sun.com/msg09568.html
   public static void convertMatrixToAxisAngle(Matrix3d rotationMatrix, AxisAngle4d axisAngle4dToPack)
   {
      Quat4d quaternion = threadLocalTemporaryQuaternion.get();
      convertMatrixToQuaternion(rotationMatrix, quaternion);
      axisAngle4dToPack.set(quaternion);
   }

   public static void convertMatrixToAxisAngle(Matrix3f rotationMatrix, AxisAngle4f axisAngle4fToPack)
   {
      Quat4d quat4d = threadLocalTemporaryQuaternion.get();
      Matrix3d rotationMatrixDouble = rotationMatrixForConvertingFloatToDouble.get();
      rotationMatrixDouble.set(rotationMatrix);
      convertMatrixToQuaternion(rotationMatrixDouble, quat4d);
      axisAngle4fToPack.set(quat4d);
   }

   /**
    * From Apache... Convert an orthogonal rotation matrix to a quaternion.
    * @param rotationMatrix orthogonal rotation matrix
    * @return quaternion corresponding to the matrix
    */
   public static void convertMatrixToQuaternion(Matrix3d rotationMatrix, Quat4d quaternionToPack)
   {
      Matrix3d orthogonalizedMatrix = tempMatrix3ds.get();

      orthogonalizeMatrix(rotationMatrix, MATRIX_TO_QUATERNION_THRESHOLD, orthogonalizedMatrix);
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

   public static void convertMatrixToQuaternion(Matrix3d rotationMatrix, Quat4f quaternionToPack)
   {
      Quat4d result = quaternionForMatrixToQuaternionFloatConvertor.get();
      convertMatrixToQuaternion(rotationMatrix, result);
      quaternionToPack.set(result);
   }

   public static void convertMatrixToYawPitchRoll(Matrix3d rotationMatrix, double[] yawPitchRollToPack)
   {
      yawPitchRollToPack[0] = computeYaw(rotationMatrix);
      yawPitchRollToPack[1] = computePitch(rotationMatrix);
      yawPitchRollToPack[2] = computeRoll(rotationMatrix);
   }

   public static void convertQuaternionToYawPitchRoll(double qx, double qy, double qz, double qw, double[] yawPitchRollToPack)
   {
      double qxqx = qx * qx;
      double qyqy = qy * qy;
      double qzqz = qz * qz;

      double pitchArgument = 2.0 * (qw * qy - qx * qz);

      yawPitchRollToPack[1] = FastMath.asin(pitchArgument);

      if (Math.abs(yawPitchRollToPack[1]) < 0.49 * Math.PI)
      {
         yawPitchRollToPack[0] = FastMath.atan2(2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qyqy + qzqz));
         yawPitchRollToPack[2] = FastMath.atan2(2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qxqx + qyqy));
      }
      else
      {
         // Here the yaw-pitch-roll representation is probably messed up. No way to get a proper yaw/roll.
         yawPitchRollToPack[0] = 2.0 * FastMath.atan2(qz, qw);
         yawPitchRollToPack[2] = 0.0;
      }
   }

   public static void convertQuaternionToYawPitchRoll(Quat4d quaternion, double[] yawPitchRollToPack)
   {
      convertQuaternionToYawPitchRoll(quaternion.x, quaternion.y, quaternion.z, quaternion.w, yawPitchRollToPack);
   }

   public static void convertQuaternionToYawPitchRoll(Quat4f quaternion, double[] yawPitchRollToPack)
   {
      convertQuaternionToYawPitchRoll(quaternion.x, quaternion.y, quaternion.z, quaternion.w, yawPitchRollToPack);
   }

   public static void convertRotationVectorToAxisAngle(Vector3d rotationVector, AxisAngle4d axisAngleToPack)
   {
      RotationVectorToAxisAngleConverter rotationVectorToAxisAngleConverter = rotationVectorToAxisAngleConvertor.get();
      rotationVectorToAxisAngleConverter.convertRotationVectorToAxisAngle(rotationVector, axisAngleToPack);
   }

   public static void convertRotationVectorToMatrix(Vector3d rotationVector, Matrix3d rotationMatrixToPack)
   {
      AxisAngle4d localAxisAngle = axisAngleForRotationVectorConvertor.get();
      convertRotationVectorToAxisAngle(rotationVector, localAxisAngle);
      rotationMatrixToPack.set(localAxisAngle);
   }

   public static void convertTransformToQuaternion(RigidBodyTransform transform, Quat4d quaternionToPack)
   {
      Matrix3d rotationMatrix = rotationMatrixForQuaternionConvertor.get();
      transform.getRotation(rotationMatrix);
      convertMatrixToQuaternion(rotationMatrix, quaternionToPack);
   }

   public static void convertTransformToYawPitchRoll(RigidBodyTransform transform, double[] yawPitchRollToPack)
   {
      Matrix3d rotationMatrix = rotationMatrixForYawPitchRollConvertor.get();
      transform.getRotation(rotationMatrix);
      convertMatrixToYawPitchRoll(rotationMatrix, yawPitchRollToPack);
   }

   /**
    * Sets the rotation matrix, based on the yaw, pitch and roll values.
    * @param yaw yaw rotation (about a fixed z-axis)
    * @param pitch pitch rotation (about a fixed y-axis)
    * @param roll roll rotation (about a fixed x-axis)
    * @param rotationMatrixToPack the rotation matrix to set, based on the yaw, pitch and roll values
    */
   public static void convertYawPitchRollToMatrix(double yaw, double pitch, double roll, Matrix3d rotationMatrixToPack)
   {
      double cosc = Math.cos(yaw);
      double sinc = Math.sin(yaw);

      double cosb = Math.cos(pitch);
      double sinb = Math.sin(pitch);

      double cosa = Math.cos(roll);
      double sina = Math.sin(roll);

      // Introduction to Robotics, 2.64
      rotationMatrixToPack.setElement(0, 0, cosc * cosb);
      rotationMatrixToPack.setElement(0, 1, cosc * sinb * sina - sinc * cosa);
      rotationMatrixToPack.setElement(0, 2, cosc * sinb * cosa + sinc * sina);

      rotationMatrixToPack.setElement(1, 0, sinc * cosb);
      rotationMatrixToPack.setElement(1, 1, sinc * sinb * sina + cosc * cosa);
      rotationMatrixToPack.setElement(1, 2, sinc * sinb * cosa - cosc * sina);

      rotationMatrixToPack.setElement(2, 0, -sinb);
      rotationMatrixToPack.setElement(2, 1, cosb * sina);
      rotationMatrixToPack.setElement(2, 2, cosb * cosa);

   }

   public static void convertYawPitchRollToMatrix(double[] yawPitchRoll, Matrix3d rotationMatrixToPack)
   {
      convertYawPitchRollToMatrix(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2], rotationMatrixToPack);
   }

   /**
    * Sets Euler parameters (unit quaternion) based on the given yaw, pitch and roll values.
    * @param yaw yaw rotation (about a fixed z-axis)
    * @param pitch pitch rotation (about a fixed y-axis)
    * @param roll roll rotation (about a fixed x-axis)
    * @param quaternionToPack Quat4d to set
    */
   public static void convertYawPitchRollToQuaternion(double yaw, double pitch, double roll, Quat4d quaternionToPack)
   {
      double halfYaw = yaw / 2.0;
      double cYaw = Math.cos(halfYaw);
      double sYaw = Math.sin(halfYaw);

      double halfPitch = pitch / 2.0;
      double cPitch = Math.cos(halfPitch);
      double sPitch = Math.sin(halfPitch);

      double halfRoll = roll / 2.0;
      double cRoll = Math.cos(halfRoll);
      double sRoll = Math.sin(halfRoll);

      quaternionToPack.w = cYaw * cPitch * cRoll + sYaw * sPitch * sRoll;
      quaternionToPack.x = cYaw * cPitch * sRoll - sYaw * sPitch * cRoll;
      quaternionToPack.y = sYaw * cPitch * sRoll + cYaw * sPitch * cRoll;
      quaternionToPack.z = sYaw * cPitch * cRoll - cYaw * sPitch * sRoll;
   }

   public static void convertYawPitchRollToQuaternion(double[] yawPitchRoll, Quat4d quaternionToPack)
   {
      convertYawPitchRollToQuaternion(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2], quaternionToPack);
   }

   public static double computeQuaternionNormSquared(Quat4d quaternion)
   {
      return square(quaternion.w) + square(quaternion.x) + square(quaternion.y) + square(quaternion.z);
   }

   public static boolean isQuaternionNormalized(Quat4d quaternion)
   {
      double normSquaredQuaternion = computeQuaternionNormSquared(quaternion);

      if (!MathTools.isFinite(normSquaredQuaternion))
         return false;
      if (!MathTools.epsilonEquals(normSquaredQuaternion, 1, 1e-5))
         return false;

      return true;
   }

   public static void checkQuaternionNormalized(Quat4d quaternion)
   {
      if (!isQuaternionNormalized(quaternion))
         throw new RuntimeException("quaternion is not normalized. quaternion is " + quaternion + " with normalization error " + Math.sqrt(computeQuaternionNormSquared(quaternion)));
   }

   public static boolean isRotationMatrixProper(Matrix3d rotationMatrix)
   {
      boolean determinantZero = Math.abs(rotationMatrix.determinant() - 1.0) < 1e-10;

      Matrix3d orthogonalityCheck = rotationMatrixForIsRotationProper.get();
      orthogonalityCheck.mulTransposeRight(rotationMatrix, rotationMatrix);

      boolean orthogonal = MatrixTools.isIdentity(orthogonalityCheck, 1e-10);
      boolean isRotationProper = orthogonal && determinantZero;

      return isRotationProper;
   }

   public static void checkProperRotationMatrix(Matrix3d rotationMatrix)
   {
      if (!isRotationMatrixProper(rotationMatrix))
         throw new RuntimeException("Matrix3d: " + rotationMatrix + " is not proper.");
   }

   /**
    * From Apache. Perfect orthogonality on a 3X3 matrix.
    * @param rotationMatrixToOrthogonalize initial matrix (not exactly orthogonal)
    * @param threshold convergence threshold for the iterative
    * orthogonality correction (convergence is reached when the
    * difference between two steps of the Frobenius norm of the
    * correction is below this threshold)
    * @return an orthogonal matrix close to m
    * @exception NotARotationMatrixException if the matrix cannot be
    * orthogonalized with the given threshold after 10 iterations
    */
   public static void orthogonalizeMatrix(Matrix3d rotationMatrixToOrthogonalize, double threshold, Matrix3d rotationMatrixResultToPack) throws NotARotationMatrixException
   {
      Matrix3d x = tempXMatrix3d.get();
      Matrix3d mx = tempMXMatrix3d.get();
      Matrix3d corr = tempCorrMatrix3d.get();

      x.transpose(rotationMatrixToOrthogonalize);

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
         rotationMatrixResultToPack.setElement(0, 0, x.m00 - 0.5 * (x.m00 * mx.m00 + x.m01 * mx.m10 + x.m02 * mx.m20 - x.m00));
         rotationMatrixResultToPack.setElement(1, 0, x.m01 - 0.5 * (x.m00 * mx.m01 + x.m01 * mx.m11 + x.m02 * mx.m21 - x.m01));
         rotationMatrixResultToPack.setElement(2, 0, x.m02 - 0.5 * (x.m00 * mx.m02 + x.m01 * mx.m12 + x.m02 * mx.m22 - x.m02));
         rotationMatrixResultToPack.setElement(0, 1, x.m10 - 0.5 * (x.m10 * mx.m00 + x.m11 * mx.m10 + x.m12 * mx.m20 - x.m10));
         rotationMatrixResultToPack.setElement(1, 1, x.m11 - 0.5 * (x.m10 * mx.m01 + x.m11 * mx.m11 + x.m12 * mx.m21 - x.m11));
         rotationMatrixResultToPack.setElement(2, 1, x.m12 - 0.5 * (x.m10 * mx.m02 + x.m11 * mx.m12 + x.m12 * mx.m22 - x.m12));
         rotationMatrixResultToPack.setElement(0, 2, x.m20 - 0.5 * (x.m20 * mx.m00 + x.m21 * mx.m10 + x.m22 * mx.m20 - x.m20));
         rotationMatrixResultToPack.setElement(1, 2, x.m21 - 0.5 * (x.m20 * mx.m01 + x.m21 * mx.m11 + x.m22 * mx.m21 - x.m21));
         rotationMatrixResultToPack.setElement(2, 2, x.m22 - 0.5 * (x.m20 * mx.m02 + x.m21 * mx.m12 + x.m22 * mx.m22 - x.m22));

         // correction on each elements
         corr.m00 = rotationMatrixResultToPack.getElement(0, 0) - x.m00;
         corr.m01 = rotationMatrixResultToPack.getElement(1, 0) - x.m01;
         corr.m02 = rotationMatrixResultToPack.getElement(2, 0) - x.m02;
         corr.m10 = rotationMatrixResultToPack.getElement(0, 1) - x.m10;
         corr.m11 = rotationMatrixResultToPack.getElement(1, 1) - x.m11;
         corr.m12 = rotationMatrixResultToPack.getElement(2, 1) - x.m12;
         corr.m20 = rotationMatrixResultToPack.getElement(0, 2) - x.m20;
         corr.m21 = rotationMatrixResultToPack.getElement(1, 2) - x.m21;
         corr.m22 = rotationMatrixResultToPack.getElement(2, 2) - x.m22;

         // Frobenius norm of the correction
         fn1 = corr.m00 * corr.m00 + corr.m01 * corr.m01 + corr.m02 * corr.m02 + corr.m10 * corr.m10 + corr.m11 * corr.m11 + corr.m12 * corr.m12 + corr.m20 * corr.m20 + corr.m21 * corr.m21 + corr.m22 * corr.m22;

         // convergence test
         if (FastMath.abs(fn1 - fn) <= threshold)
         {
            return;
         }

         // prepare next iteration
         x.m00 = rotationMatrixResultToPack.getElement(0, 0);
         x.m01 = rotationMatrixResultToPack.getElement(1, 0);
         x.m02 = rotationMatrixResultToPack.getElement(2, 0);
         x.m10 = rotationMatrixResultToPack.getElement(0, 1);
         x.m11 = rotationMatrixResultToPack.getElement(1, 1);
         x.m12 = rotationMatrixResultToPack.getElement(2, 1);
         x.m20 = rotationMatrixResultToPack.getElement(0, 2);
         x.m21 = rotationMatrixResultToPack.getElement(1, 2);
         x.m22 = rotationMatrixResultToPack.getElement(2, 2);
         fn = fn1;
      }

      // the algorithm did not converge after 10 iterations
      throw new NotARotationMatrixException(LocalizedFormats.UNABLE_TO_ORTHOGONOLIZE_MATRIX, i - 1);
   }

   public static boolean quaternionEpsilonEquals(Quat4d original, Quat4d result, double epsilon)
   {
      if (original.epsilonEquals(result, epsilon))
      {
         return true;
      }
      else
      {
         Quat4d originalNegated = quaternionForEpsilonEquals.get();
         originalNegated.negate(original);

         return originalNegated.epsilonEquals(result, epsilon);
      }
   }

   public static boolean quaternionEpsilonEquals(Quat4f original, Quat4f result, float epsilon)
   {
      if (original.epsilonEquals(result, epsilon))
      {
         return true;
      }
      else
      {
         Quat4f originalNegated = quaternionFloatForEpsilonEquals.get();
         originalNegated.negate(original);

         return originalNegated.epsilonEquals(result, epsilon);
      }
   }

   public static void trimAxisAngleMinusPiToPi(AxisAngle4d axisAngle4d, AxisAngle4d axisAngleTrimmedToPack)
   {
      axisAngleTrimmedToPack.set(axisAngle4d);
      axisAngleTrimmedToPack.angle = AngleTools.trimAngleMinusPiToPi(axisAngleTrimmedToPack.angle);
   }

   public static boolean axisAngleEpsilonEquals(AxisAngle4d original, AxisAngle4d result, double epsilon, AxisAngleComparisonMode mode)
   {
      if (mode == AxisAngleComparisonMode.IGNORE_FLIPPED_AXES)
      {
         return axisAngleEpsilonEqualsIgnoreFlippedAxes(original, result, epsilon);
      }
      else if (mode == AxisAngleComparisonMode.IGNORE_FLIPPED_AXES_ROTATION_DIRECTION_AND_COMPLETE_ROTATIONS)
      {
         AxisAngle4d originalMinusPiToPi = originalAxisAngleForEpsilonEquals.get();
         trimAxisAngleMinusPiToPi(original, originalMinusPiToPi);
         AxisAngle4d resultMinusPiToPi = resultAxisAngleForEpsilonEquals.get();
         trimAxisAngleMinusPiToPi(result, resultMinusPiToPi);

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

   public static boolean axisAngleEpsilonEqualsIgnoreFlippedAxes(AxisAngle4d original, AxisAngle4d result, double epsilon)
   {
      if (original.epsilonEquals(result, epsilon))
      {
         return true;
      }
      else
      {
         AxisAngle4d originalNegated = originalAxisAngleForEpsilonEquals.get();
         originalNegated.set(original);
         originalNegated.angle *= -1.0;
         originalNegated.x *= -1.0;
         originalNegated.y *= -1.0;
         originalNegated.z *= -1.0;

         return originalNegated.epsilonEquals(result, epsilon);
      }
   }

   // FIXME This needs to get moved to MatrixTools.
   @Deprecated
   public static void quaternionToMatrix(DenseMatrix64F matrix, Quat4d quaternion, int rowStart)
   {
      int index = rowStart;
      matrix.set(index++, 0, quaternion.getX());
      matrix.set(index++, 0, quaternion.getY());
      matrix.set(index++, 0, quaternion.getZ());
      matrix.set(index++, 0, quaternion.getW());
   }

   // FIXME This needs to get moved to MatrixTools.
   @Deprecated
   public static void matrixToQuaternion(Quat4d quaternion, DenseMatrix64F matrix, int rowStart)
   {
      int index = rowStart;
      double x = matrix.get(index++, 0);
      double y = matrix.get(index++, 0);
      double z = matrix.get(index++, 0);
      double w = matrix.get(index++, 0);
      quaternion.set(x, y, z, w);
   }
}
