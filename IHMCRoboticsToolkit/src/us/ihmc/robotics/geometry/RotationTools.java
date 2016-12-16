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
      double yaw = Math.atan2(rotationMatrix.getM10(), rotationMatrix.getM00());
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
      double pitch = Math.asin(-rotationMatrix.getM20());
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
      double roll = Math.atan2(rotationMatrix.getM21(), rotationMatrix.getM22());
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
      if (Math.abs(zNormal.getZ()) < 1e-9)
      {
         quaternionToPack.set(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
         return;
      }
      double Cz = -1.0 * (zNormal.getX() + Cy * zNormal.getY()) / zNormal.getZ();
      double CT = Math.sqrt(Cx * Cx + Cy * Cy + Cz * Cz);
      if (CT < 1e-9)
         throw new RuntimeException("Error calculating Quaternion");

      Vector3d xAxis = new Vector3d(Cx / CT, Cy / CT, Cz / CT);
      if (xAxis.getX() * Math.cos(yaw) + xAxis.getY() * Math.sin(yaw) < 0.0)
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
         quaternionToPack.setW(0.5 * FastMath.sqrt(s + 1.0));
         double inv = 0.25 / quaternionToPack.getW();
         quaternionToPack.setX(inv * (rotationMatrix.getElement(2, 1) - rotationMatrix.getElement(1, 2)));
         quaternionToPack.setY(inv * (rotationMatrix.getElement(0, 2) - rotationMatrix.getElement(2, 0)));
         quaternionToPack.setZ(inv * (rotationMatrix.getElement(1, 0) - rotationMatrix.getElement(0, 1)));
      }
      else
      {
         s = rotationMatrix.getElement(0, 0) - rotationMatrix.getElement(1, 1) - rotationMatrix.getElement(2, 2);

         if (s > -0.19)
         {
            // compute q1 and deduce q0, q2 and q3
            quaternionToPack.setX(0.5 * FastMath.sqrt(s + 1.0));
            double inv = 0.25 / quaternionToPack.getX();
            quaternionToPack.setW(inv * (rotationMatrix.getElement(2, 1) - rotationMatrix.getElement(1, 2)));
            quaternionToPack.setY(inv * (rotationMatrix.getElement(1, 0) + rotationMatrix.getElement(0, 1)));
            quaternionToPack.setZ(inv * (rotationMatrix.getElement(2, 0) + rotationMatrix.getElement(0, 2)));
         }
         else
         {
            s = rotationMatrix.getElement(1, 1) - rotationMatrix.getElement(0, 0) - rotationMatrix.getElement(2, 2);

            if (s > -0.19)
            {
               // compute q2 and deduce q0, q1 and q3
               quaternionToPack.setY(0.5 * FastMath.sqrt(s + 1.0));
               double inv = 0.25 / quaternionToPack.getY();
               quaternionToPack.setW(inv * (rotationMatrix.getElement(0, 2) - rotationMatrix.getElement(2, 0)));
               quaternionToPack.setX(inv * (rotationMatrix.getElement(1, 0) + rotationMatrix.getElement(0, 1)));
               quaternionToPack.setZ(inv * (rotationMatrix.getElement(1, 2) + rotationMatrix.getElement(2, 1)));
            }
            else
            {
               // compute q3 and deduce q0, q1 and q2
               s = rotationMatrix.getElement(2, 2) - rotationMatrix.getElement(0, 0) - rotationMatrix.getElement(1, 1);
               quaternionToPack.setZ(0.5 * FastMath.sqrt(s + 1.0));
               double inv = 0.25 / quaternionToPack.getZ();
               quaternionToPack.setW(inv * (rotationMatrix.getElement(1, 0) - rotationMatrix.getElement(0, 1)));
               quaternionToPack.setX(inv * (rotationMatrix.getElement(2, 0) + rotationMatrix.getElement(0, 2)));
               quaternionToPack.setY(inv * (rotationMatrix.getElement(1, 2) + rotationMatrix.getElement(2, 1)));
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

   static double closest(double org, double ref) {
      double result = org;
      double min_diff = Math.abs(result-ref);
      for(int i=1; true; i++) {
         double my_angle = org + i*Math.PI;
         double my_diff = Math.abs(my_angle-ref);
         if(my_diff < min_diff) {
            result = my_angle;
            min_diff = my_diff;
         }
         else break;
      }
      for(int i=-1; true; i--) {
         double my_angle = org + i*Math.PI;
         double my_diff = Math.abs(my_angle-ref);
         if(my_diff < min_diff) {
            result = my_angle;
            min_diff = my_diff;
         }
         else break;
      }
      return result;
   }

   public static void convertMatrixToClosestYawPitchRoll(Matrix3d rotationMatrix, double[] yawPitchRollRef, double[] yawPitchRollToPack) {
      double yawRef = yawPitchRollRef[0];
      double pitchRef = yawPitchRollRef[1];
      double rollRef = yawPitchRollRef[2];
      double m20 = rotationMatrix.getM20();
      m20 = Math.min(m20, 1.0);
      m20 = Math.max(m20, -1.0);
      double pitch1 = Math.asin(-m20);
      if (Double.isNaN(pitch1))
      {
         throw new RuntimeException("Pitch is NaN! rotationMatrix = " + rotationMatrix);
      }
      double pitch2 = Math.PI - pitch1;
      double sinp = -m20;
      double cosp1 = Math.cos(pitch1);
      double cosp2 = Math.cos(pitch2);

      double pitch = 0.0;
      double yaw = 0.0;
      double roll = 0.0;

      if(Math.abs(cosp1) > 0.1) {
         double yaw1 = Math.atan2(rotationMatrix.getM10()/cosp1, rotationMatrix.getM00()/cosp1);
         double roll1 = Math.atan2(rotationMatrix.getM21()/cosp1, rotationMatrix.getM22()/cosp1);
         double diff1 = (yaw1-yawRef)*(yaw1-yawRef) + (pitch1-pitchRef)*(pitch1-pitchRef) + (roll1-rollRef)*(roll1-rollRef);
         double yaw2 = Math.atan2(rotationMatrix.getM10()/cosp2, rotationMatrix.getM00()/cosp2);
         double roll2 = Math.atan2(rotationMatrix.getM21()/cosp2, rotationMatrix.getM22()/cosp2);
         double diff2 = (yaw2-yawRef)*(yaw2-yawRef) + (pitch2-pitchRef)*(pitch2-pitchRef) + (roll2-rollRef)*(roll2-rollRef);
         if(diff1 < diff2) {
            yaw = yaw1;
            pitch = pitch1;
            roll = roll1;
         }
         else {
            yaw = yaw2;
            pitch = pitch2;
            roll = roll2;
         }
      }
      else {
         // close to singularity
         double m01 = rotationMatrix.getM01();
         double m02 = rotationMatrix.getM02();
         double m11 = rotationMatrix.getM11();
         double m12 = rotationMatrix.getM12();

         double c00 = m01*m01 + m02*m02 - sinp*sinp;
         double c01 = m11*m01 + m12*m02;
         double c10 = c01;
         double c11 = m11*m11 + m12*m12 - sinp*sinp;

         double a00 = m12*m12 + m02*m02 - sinp*sinp;
         double a01 = m01*m02 + m11*m12;
         double a10 = a01;
         double a11 = m01*m01* m11*m11 - sinp*sinp;

         int index = 0;
         double max_value = Math.abs(c01);
         if(Math.abs(c11) > max_value) {
            max_value = Math.abs(c11);
            index = 1;
         }
         if(Math.abs(a01) > max_value) {
            max_value = Math.abs(a01);
            index = 2;
         }
         if(Math.abs(a11) > max_value) {
            max_value = Math.abs(a11);
            index = 3;
         }
         double pitch0 = 0.0, yaw0 = 0.0, roll0 = 0.0;
         if (index == 0 || index == 1) {
            if (index == 0) {
               yaw0 = closest(Math.atan2(c00, -c01), yawRef);
            } else if (index == 1) {
               yaw0 = closest(Math.atan2(c10, -c11), yawRef);
            }
            double sinc = Math.sin(yaw0);
            double cosc = Math.cos(yaw0);
            double sina = (cosc * m01 + sinc * m11) / sinp;
            double cosa = (cosc * m02 + sinc * m12) / sinp;
            roll0 = closest(Math.atan2(sina, cosa), rollRef);
         }
         else {
            if (index == 2) {
               roll0 = closest(Math.atan2(a00, -a01), rollRef);
            } else if (index == 3) {
               roll0 = closest(Math.atan2(a10, -a11), rollRef);
            }
            double sina = Math.sin(roll0);
            double cosa = Math.cos(roll0);
            double sinc = (sina*m11 + cosa*m12) / sinp;
            double cosc = (sina*m01 + cosa*m02) / sinp;
            yaw0 = closest(Math.atan2(sinc, cosc), yawRef);
         }
         {
            double sina = Math.sin(roll0);
            double cosa = Math.cos(roll0);
            double sinc = Math.sin(yaw0);
            double cosc = Math.cos(yaw0);
            int index0 = 0;
            double max_val = Math.abs(sina);
            if(Math.abs(cosa) > max_val) {
               index0 = 1;
               max_val = Math.abs(cosa);
            }
            if(Math.abs(sinc) > max_val) {
               index0 = 2;
               max_val = Math.abs(sinc);
            }
            if(Math.abs(cosc) > max_val) {
               index0 = 3;
               max_val = Math.abs(cosc);
            }
            double cosp = 0.0;
            if(index0 == 0) {
               cosp = rotationMatrix.getM21() / sina;
            }
            else if(index0 == 1) {
               cosp = rotationMatrix.getM22() / cosa;
            }
            else if(index0 == 3) {
               cosp = rotationMatrix.getM00() / cosc;
            }
            else {
               cosp = rotationMatrix.getM10() / sinc;
            }
            pitch0 = Math.atan2(-rotationMatrix.getM20(), cosp);
         }
         yaw = yaw0;
         roll = roll0;
         pitch = pitch0;
      }

      if (Double.isNaN(yaw))
      {
         throw new RuntimeException("Yaw is NaN! rotationMatrix = " + rotationMatrix);
      }
      if (Double.isNaN(roll))
      {
         throw new RuntimeException("Roll is NaN! rotationMatrix = " + rotationMatrix);
      }
      yawPitchRollToPack[0] = yaw;
      yawPitchRollToPack[1] = pitch;
      yawPitchRollToPack[2] = roll;
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
      convertQuaternionToYawPitchRoll(quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getW(), yawPitchRollToPack);
   }

   public static void convertQuaternionToYawPitchRoll(Quat4f quaternion, double[] yawPitchRollToPack)
   {
      convertQuaternionToYawPitchRoll(quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getW(), yawPitchRollToPack);
   }

   public static void convertQuaternionToRotationVector(Quat4d quaternion, Vector3d rotationVectorToPack)
   {
      AxisAngle4d axisAngle = axisAngleForRotationVectorConvertor.get();
      axisAngle.set(quaternion);
      rotationVectorToPack.set(axisAngle.getX(), axisAngle.getY(), axisAngle.getZ());
      rotationVectorToPack.scale(axisAngle.getAngle());
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

   public static void convertRotationVectorToQuaternion(Vector3d rotationVector, Quat4d quaternionToPack)
   {
      AxisAngle4d localAxisAngle = axisAngleForRotationVectorConvertor.get();
      convertRotationVectorToAxisAngle(rotationVector, localAxisAngle);
      quaternionToPack.set(localAxisAngle);
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

   public static void convertTransformToClosestYawPitchRoll(RigidBodyTransform transform, double[] yawPitchRollRef, double[] yawPitchRollToPack)
   {
      Matrix3d rotationMatrix = rotationMatrixForYawPitchRollConvertor.get();
      transform.getRotation(rotationMatrix);
      convertMatrixToClosestYawPitchRoll(rotationMatrix, yawPitchRollRef, yawPitchRollToPack);
   }

   /**
    * Removes the pitch and roll from a RigidBodyTransform. Preserves the yaw.
    * When done, the xAxis should still point in the same direction from the point of view of looking down (-z) 
    * @param transform
    */
   public static void removePitchAndRollFromTransform(RigidBodyTransform transform)
   {
      if (Math.abs(transform.getM22() - 1.0) < 1e-4) return;
      double m00 = transform.getM00();
      double m10 = transform.getM10();

      double magnitude = Math.sqrt(m00*m00 + m10*m10);
      m00 = m00 / magnitude;
      m10 = m10 / magnitude;

      transform.setM00(m00);
      transform.setM10(m10);
      transform.setM20(0.0);

      transform.setM01(-m10);
      transform.setM11(m00);
      transform.setM21(0.0);

      transform.setM02(0.0);
      transform.setM12(0.0);
      transform.setM22(1.0);
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

      quaternionToPack.setW(cYaw * cPitch * cRoll + sYaw * sPitch * sRoll);
      quaternionToPack.setX(cYaw * cPitch * sRoll - sYaw * sPitch * cRoll);
      quaternionToPack.setY(sYaw * cPitch * sRoll + cYaw * sPitch * cRoll);
      quaternionToPack.setZ(sYaw * cPitch * cRoll - cYaw * sPitch * sRoll);
   }

   public static void convertYawPitchRollToQuaternion(double[] yawPitchRoll, Quat4d quaternionToPack)
   {
      convertYawPitchRollToQuaternion(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2], quaternionToPack);
   }

   private static final ThreadLocal<Vector3d> angularVelocityForIntegrator = new ThreadLocal<Vector3d>()
   {
      @Override
      public Vector3d initialValue()
      {
         return new Vector3d();
      }
   };

   private static final ThreadLocal<AxisAngle4d> axisAngleForIntegrator = new ThreadLocal<AxisAngle4d>()
   {
      @Override
      public AxisAngle4d initialValue()
      {
         return new AxisAngle4d();
      }
   };

   public static void integrateAngularVelocity(FrameVector angularVelocityToIntegrate, double integrationTime, FrameOrientation orientationResultToPack)
   {
      AxisAngle4d axisAngleResult = axisAngleForIntegrator.get();
      integrateAngularVelocity(angularVelocityToIntegrate.getVector(), integrationTime, axisAngleResult);
      orientationResultToPack.setIncludingFrame(angularVelocityToIntegrate.getReferenceFrame(), axisAngleResult);
   }

   public static void integrateAngularVelocity(Vector3d angularVelocityToIntegrate, double integrationTime, Matrix3d orientationResultToPack)
   {
      AxisAngle4d axisAngleResult = axisAngleForIntegrator.get();
      integrateAngularVelocity(angularVelocityToIntegrate, integrationTime, axisAngleResult);
      orientationResultToPack.set(axisAngleResult);
   }

   public static void integrateAngularVelocity(Vector3d angularVelocityToIntegrate, double integrationTime, Quat4d orientationResultToPack)
   {
      AxisAngle4d axisAngleResult = axisAngleForIntegrator.get();
      integrateAngularVelocity(angularVelocityToIntegrate, integrationTime, axisAngleResult);
      orientationResultToPack.set(axisAngleResult);
   }

   public static void integrateAngularVelocity(Vector3d angularVelocityToIntegrate, double integrationTime, AxisAngle4d orientationResultToPack)
   {
      Vector3d angularVelocityIntegrated = angularVelocityForIntegrator.get();
      angularVelocityIntegrated.set(angularVelocityToIntegrate);
      angularVelocityIntegrated.scale(integrationTime);
      convertRotationVectorToAxisAngle(angularVelocityIntegrated, orientationResultToPack);
   }

   public static double computeQuaternionNormSquared(Quat4d quaternion)
   {
      return square(quaternion.getW()) + square(quaternion.getX()) + square(quaternion.getY()) + square(quaternion.getZ());
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
         mx.setM00(x.getM00() * x.getM00() + x.getM10() * x.getM10() + x.getM20() * x.getM20());
         mx.setM10(x.getM01() * x.getM00() + x.getM11() * x.getM10() + x.getM21() * x.getM20());
         mx.setM20(x.getM02() * x.getM00() + x.getM12() * x.getM10() + x.getM22() * x.getM20());
         mx.setM01(x.getM00() * x.getM01() + x.getM10() * x.getM11() + x.getM20() * x.getM21());
         mx.setM11(x.getM01() * x.getM01() + x.getM11() * x.getM11() + x.getM21() * x.getM21());
         mx.setM21(x.getM02() * x.getM01() + x.getM12() * x.getM11() + x.getM22() * x.getM21());
         mx.setM02(x.getM00() * x.getM02() + x.getM10() * x.getM12() + x.getM20() * x.getM22());
         mx.setM12(x.getM01() * x.getM02() + x.getM11() * x.getM12() + x.getM21() * x.getM22());
         mx.setM22(x.getM02() * x.getM02() + x.getM12() * x.getM12() + x.getM22() * x.getM22());

         // Xn+1
         rotationMatrixResultToPack.setElement(0, 0, x.getM00() - 0.5 * (x.getM00() * mx.getM00() + x.getM01() * mx.getM10() + x.getM02() * mx.getM20() - x.getM00()));
         rotationMatrixResultToPack.setElement(1, 0, x.getM01() - 0.5 * (x.getM00() * mx.getM01() + x.getM01() * mx.getM11() + x.getM02() * mx.getM21() - x.getM01()));
         rotationMatrixResultToPack.setElement(2, 0, x.getM02() - 0.5 * (x.getM00() * mx.getM02() + x.getM01() * mx.getM12() + x.getM02() * mx.getM22() - x.getM02()));
         rotationMatrixResultToPack.setElement(0, 1, x.getM10() - 0.5 * (x.getM10() * mx.getM00() + x.getM11() * mx.getM10() + x.getM12() * mx.getM20() - x.getM10()));
         rotationMatrixResultToPack.setElement(1, 1, x.getM11() - 0.5 * (x.getM10() * mx.getM01() + x.getM11() * mx.getM11() + x.getM12() * mx.getM21() - x.getM11()));
         rotationMatrixResultToPack.setElement(2, 1, x.getM12() - 0.5 * (x.getM10() * mx.getM02() + x.getM11() * mx.getM12() + x.getM12() * mx.getM22() - x.getM12()));
         rotationMatrixResultToPack.setElement(0, 2, x.getM20() - 0.5 * (x.getM20() * mx.getM00() + x.getM21() * mx.getM10() + x.getM22() * mx.getM20() - x.getM20()));
         rotationMatrixResultToPack.setElement(1, 2, x.getM21() - 0.5 * (x.getM20() * mx.getM01() + x.getM21() * mx.getM11() + x.getM22() * mx.getM21() - x.getM21()));
         rotationMatrixResultToPack.setElement(2, 2, x.getM22() - 0.5 * (x.getM20() * mx.getM02() + x.getM21() * mx.getM12() + x.getM22() * mx.getM22() - x.getM22()));

         // correction on each elements
         corr.setM00(rotationMatrixResultToPack.getElement(0, 0) - x.getM00());
         corr.setM01(rotationMatrixResultToPack.getElement(1, 0) - x.getM01());
         corr.setM02(rotationMatrixResultToPack.getElement(2, 0) - x.getM02());
         corr.setM10(rotationMatrixResultToPack.getElement(0, 1) - x.getM10());
         corr.setM11(rotationMatrixResultToPack.getElement(1, 1) - x.getM11());
         corr.setM12(rotationMatrixResultToPack.getElement(2, 1) - x.getM12());
         corr.setM20(rotationMatrixResultToPack.getElement(0, 2) - x.getM20());
         corr.setM21(rotationMatrixResultToPack.getElement(1, 2) - x.getM21());
         corr.setM22(rotationMatrixResultToPack.getElement(2, 2) - x.getM22());

         // Frobenius norm of the correction
         fn1 = corr.getM00() * corr.getM00() + corr.getM01() * corr.getM01() + corr.getM02() * corr.getM02() + corr.getM10() * corr.getM10() + corr.getM11() * corr.getM11() + corr.getM12() * corr.getM12() + corr.getM20() * corr.getM20() + corr.getM21() * corr.getM21() + corr.getM22() * corr.getM22();

         // convergence test
         if (FastMath.abs(fn1 - fn) <= threshold)
         {
            return;
         }

         // prepare next iteration
         x.setM00(rotationMatrixResultToPack.getElement(0, 0));
         x.setM01(rotationMatrixResultToPack.getElement(1, 0));
         x.setM02(rotationMatrixResultToPack.getElement(2, 0));
         x.setM10(rotationMatrixResultToPack.getElement(0, 1));
         x.setM11(rotationMatrixResultToPack.getElement(1, 1));
         x.setM12(rotationMatrixResultToPack.getElement(2, 1));
         x.setM20(rotationMatrixResultToPack.getElement(0, 2));
         x.setM21(rotationMatrixResultToPack.getElement(1, 2));
         x.setM22(rotationMatrixResultToPack.getElement(2, 2));
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
      axisAngleTrimmedToPack.setAngle(AngleTools.trimAngleMinusPiToPi(axisAngleTrimmedToPack.getAngle()));
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

         boolean originalAxisAngleIsZero = MathTools.epsilonEquals(originalMinusPiToPi.getAngle(), 0.0, 0.1 * epsilon);
         boolean resultAxisAngleIsZero = MathTools.epsilonEquals(resultMinusPiToPi.getAngle(), 0.0, 0.1 * epsilon);

         boolean originalAngleIs180 = MathTools.epsilonEquals(Math.abs(originalMinusPiToPi.getAngle()), Math.PI, 0.1 * epsilon);
         boolean resultAngleIs180 = MathTools.epsilonEquals(Math.abs(resultMinusPiToPi.getAngle()), Math.PI, 0.1 * epsilon);

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
         originalNegated.setAngle(originalNegated.getAngle() * -1.0);
         originalNegated.setX(originalNegated.getX() * -1.0);
         originalNegated.setY(originalNegated.getY() * -1.0);
         originalNegated.setZ(originalNegated.getZ() * -1.0);

         return originalNegated.epsilonEquals(result, epsilon);
      }
   }
}
