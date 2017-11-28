package us.ihmc.robotics.geometry;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.rotationConversion.AxisAngleConversion;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.commons.MathTools;

public class RotationTools
{
   public enum AxisAngleComparisonMode
   {
      IGNORE_FLIPPED_AXES, IGNORE_FLIPPED_AXES_ROTATION_DIRECTION_AND_COMPLETE_ROTATIONS;
   }

   public static final int QUATERNION_SIZE = 4;

   public static final double MATRIX_TO_QUATERNION_THRESHOLD = 1e-10;

   private static final ThreadLocal<RotationMatrix> rotationMatrixForYawPitchRollConvertor = new ThreadLocal<RotationMatrix>()
   {
      @Override
      public RotationMatrix initialValue()
      {
         return new RotationMatrix();
      }
   };

   private static final ThreadLocal<Quaternion> quaternionForEpsilonEquals = new ThreadLocal<Quaternion>()
   {
      @Override
      public Quaternion initialValue()
      {
         return new Quaternion();
      }
   };

   private static final ThreadLocal<AxisAngle> originalAxisAngleForEpsilonEquals = new ThreadLocal<AxisAngle>()
   {
      @Override
      public AxisAngle initialValue()
      {
         return new AxisAngle();
      }
   };

   private static final ThreadLocal<AxisAngle> resultAxisAngleForEpsilonEquals = new ThreadLocal<AxisAngle>()
   {
      @Override
      public AxisAngle initialValue()
      {
         return new AxisAngle();
      }
   };

   private static ThreadLocal<RotationMatrix> rotationMatrixForQuaternionFromYawAndZNormal = new ThreadLocal<RotationMatrix>()
   {
      @Override
      protected RotationMatrix initialValue()
      {
         return new RotationMatrix();
      }
   };

   public static void computeQuaternionFromYawAndZNormal(double yaw, Vector3DReadOnly zNormal, QuaternionBasics quaternionToPack)
   {
      double Cx = 1.0;
      double Cy = Math.tan(yaw);
      if (Math.abs(zNormal.getZ()) < 1e-9)
      {
         quaternionToPack.setToNaN();
         return;
      }
      double Cz = -1.0 * (zNormal.getX() + Cy * zNormal.getY()) / zNormal.getZ();
      double CT = Math.sqrt(Cx * Cx + Cy * Cy + Cz * Cz);
      if (CT < 1e-9)
         throw new RuntimeException("Error calculating Quaternion");

      Vector3D xAxis = new Vector3D(Cx / CT, Cy / CT, Cz / CT);
      if (xAxis.getX() * Math.cos(yaw) + xAxis.getY() * Math.sin(yaw) < 0.0)
      {
         xAxis.negate();
      }
      Vector3D yAxis = new Vector3D();
      Vector3DReadOnly zAxis = zNormal;
      yAxis.cross(zAxis, xAxis);

      RotationMatrix rotationMatrix = rotationMatrixForQuaternionFromYawAndZNormal.get();
      rotationMatrix.setColumns(xAxis, yAxis, zAxis);

      quaternionToPack.set(rotationMatrix);
   }

   static double closest(double org, double ref)
   {
      double result = org;
      double min_diff = Math.abs(result - ref);
      for (int i = 1; true; i++)
      {
         double my_angle = org + i * Math.PI;
         double my_diff = Math.abs(my_angle - ref);
         if (my_diff < min_diff)
         {
            result = my_angle;
            min_diff = my_diff;
         }
         else
            break;
      }
      for (int i = -1; true; i--)
      {
         double my_angle = org + i * Math.PI;
         double my_diff = Math.abs(my_angle - ref);
         if (my_diff < min_diff)
         {
            result = my_angle;
            min_diff = my_diff;
         }
         else
            break;
      }
      return result;
   }

   public static void convertMatrixToClosestYawPitchRoll(RotationMatrix rotationMatrix, double[] yawPitchRollRef, double[] yawPitchRollToPack)
   {
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

      if (Math.abs(cosp1) > 0.1)
      {
         double yaw1 = Math.atan2(rotationMatrix.getM10() / cosp1, rotationMatrix.getM00() / cosp1);
         double roll1 = Math.atan2(rotationMatrix.getM21() / cosp1, rotationMatrix.getM22() / cosp1);
         double diff1 = (yaw1 - yawRef) * (yaw1 - yawRef) + (pitch1 - pitchRef) * (pitch1 - pitchRef) + (roll1 - rollRef) * (roll1 - rollRef);
         double yaw2 = Math.atan2(rotationMatrix.getM10() / cosp2, rotationMatrix.getM00() / cosp2);
         double roll2 = Math.atan2(rotationMatrix.getM21() / cosp2, rotationMatrix.getM22() / cosp2);
         double diff2 = (yaw2 - yawRef) * (yaw2 - yawRef) + (pitch2 - pitchRef) * (pitch2 - pitchRef) + (roll2 - rollRef) * (roll2 - rollRef);
         if (diff1 < diff2)
         {
            yaw = yaw1;
            pitch = pitch1;
            roll = roll1;
         }
         else
         {
            yaw = yaw2;
            pitch = pitch2;
            roll = roll2;
         }
      }
      else
      {
         // close to singularity
         double m01 = rotationMatrix.getM01();
         double m02 = rotationMatrix.getM02();
         double m11 = rotationMatrix.getM11();
         double m12 = rotationMatrix.getM12();

         double c00 = m01 * m01 + m02 * m02 - sinp * sinp;
         double c01 = m11 * m01 + m12 * m02;
         double c10 = c01;
         double c11 = m11 * m11 + m12 * m12 - sinp * sinp;

         double a00 = m12 * m12 + m02 * m02 - sinp * sinp;
         double a01 = m01 * m02 + m11 * m12;
         double a10 = a01;
         double a11 = m01 * m01 * m11 * m11 - sinp * sinp;

         int index = 0;
         double max_value = Math.abs(c01);
         if (Math.abs(c11) > max_value)
         {
            max_value = Math.abs(c11);
            index = 1;
         }
         if (Math.abs(a01) > max_value)
         {
            max_value = Math.abs(a01);
            index = 2;
         }
         if (Math.abs(a11) > max_value)
         {
            max_value = Math.abs(a11);
            index = 3;
         }
         double pitch0 = 0.0, yaw0 = 0.0, roll0 = 0.0;
         if (index == 0 || index == 1)
         {
            if (index == 0)
            {
               yaw0 = closest(Math.atan2(c00, -c01), yawRef);
            }
            else if (index == 1)
            {
               yaw0 = closest(Math.atan2(c10, -c11), yawRef);
            }
            double sinc = Math.sin(yaw0);
            double cosc = Math.cos(yaw0);
            double sina = (cosc * m01 + sinc * m11) / sinp;
            double cosa = (cosc * m02 + sinc * m12) / sinp;
            roll0 = closest(Math.atan2(sina, cosa), rollRef);
         }
         else
         {
            if (index == 2)
            {
               roll0 = closest(Math.atan2(a00, -a01), rollRef);
            }
            else if (index == 3)
            {
               roll0 = closest(Math.atan2(a10, -a11), rollRef);
            }
            double sina = Math.sin(roll0);
            double cosa = Math.cos(roll0);
            double sinc = (sina * m11 + cosa * m12) / sinp;
            double cosc = (sina * m01 + cosa * m02) / sinp;
            yaw0 = closest(Math.atan2(sinc, cosc), yawRef);
         }
         {
            double sina = Math.sin(roll0);
            double cosa = Math.cos(roll0);
            double sinc = Math.sin(yaw0);
            double cosc = Math.cos(yaw0);
            int index0 = 0;
            double max_val = Math.abs(sina);
            if (Math.abs(cosa) > max_val)
            {
               index0 = 1;
               max_val = Math.abs(cosa);
            }
            if (Math.abs(sinc) > max_val)
            {
               index0 = 2;
               max_val = Math.abs(sinc);
            }
            if (Math.abs(cosc) > max_val)
            {
               index0 = 3;
               max_val = Math.abs(cosc);
            }
            double cosp = 0.0;
            if (index0 == 0)
            {
               cosp = rotationMatrix.getM21() / sina;
            }
            else if (index0 == 1)
            {
               cosp = rotationMatrix.getM22() / cosa;
            }
            else if (index0 == 3)
            {
               cosp = rotationMatrix.getM00() / cosc;
            }
            else
            {
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

   public static void convertTransformToClosestYawPitchRoll(RigidBodyTransform transform, double[] yawPitchRollRef, double[] yawPitchRollToPack)
   {
      RotationMatrix rotationMatrix = rotationMatrixForYawPitchRollConvertor.get();
      transform.getRotation(rotationMatrix);
      convertMatrixToClosestYawPitchRoll(rotationMatrix, yawPitchRollRef, yawPitchRollToPack);
   }

   /**
    * Removes the pitch and roll from a RigidBodyTransform. Preserves the yaw. When done, the xAxis
    * should still point in the same direction from the point of view of looking down (-z)
    * 
    * @param transform
    */
   public static void removePitchAndRollFromTransform(RigidBodyTransform transform)
   {
      if (Math.abs(transform.getM22() - 1.0) < 1e-4)
         return;
      double m00 = transform.getM00();
      double m10 = transform.getM10();

      double magnitude = Math.sqrt(m00 * m00 + m10 * m10);
      m00 = m00 / magnitude;
      m10 = m10 / magnitude;

      transform.setRotation(m00, -m10, 0.0, m10, m00, 0.0, 0.0, 0.0, 1.0);
   }

   private static final ThreadLocal<Vector3D> angularVelocityForIntegrator = new ThreadLocal<Vector3D>()
   {
      @Override
      public Vector3D initialValue()
      {
         return new Vector3D();
      }
   };

   private static final ThreadLocal<AxisAngle> axisAngleForIntegrator = new ThreadLocal<AxisAngle>()
   {
      @Override
      public AxisAngle initialValue()
      {
         return new AxisAngle();
      }
   };

   public static void integrateAngularVelocity(FrameVector3D angularVelocityToIntegrate, double integrationTime, FrameQuaternion orientationResultToPack)
   {
      AxisAngle axisAngleResult = axisAngleForIntegrator.get();
      integrateAngularVelocity(angularVelocityToIntegrate.getVector(), integrationTime, axisAngleResult);
      orientationResultToPack.setIncludingFrame(angularVelocityToIntegrate.getReferenceFrame(), axisAngleResult);
   }

   public static void integrateAngularVelocity(Vector3DReadOnly angularVelocityToIntegrate, double integrationTime, RotationMatrix orientationResultToPack)
   {
      AxisAngle axisAngleResult = axisAngleForIntegrator.get();
      integrateAngularVelocity(angularVelocityToIntegrate, integrationTime, axisAngleResult);
      orientationResultToPack.set(axisAngleResult);
   }

   public static void integrateAngularVelocity(Vector3DReadOnly angularVelocityToIntegrate, double integrationTime, QuaternionBasics orientationResultToPack)
   {
      AxisAngle axisAngleResult = axisAngleForIntegrator.get();
      integrateAngularVelocity(angularVelocityToIntegrate, integrationTime, axisAngleResult);
      orientationResultToPack.set(axisAngleResult);
   }

   public static void integrateAngularVelocity(Vector3DReadOnly angularVelocityToIntegrate, double integrationTime, AxisAngleBasics orientationResultToPack)
   {
      Vector3D angularVelocityIntegrated = angularVelocityForIntegrator.get();
      angularVelocityIntegrated.set(angularVelocityToIntegrate);
      angularVelocityIntegrated.scale(integrationTime);
      AxisAngleConversion.convertRotationVectorToAxisAngle(angularVelocityIntegrated, orientationResultToPack);
   }

   /**
    * Computes the angular velocity vector from the time derivatives of the yaw-pitch-roll angles.
    * <p>
    * The resulting velocity is expressed in the local coordinate system described by the
    * yaw-pitch-roll angles.
    * </p>
    * 
    * @param yaw the current rotation angle about the z-axis.
    * @param pitch the current rotation angle about the y-axis.
    * @param roll the current rotation angle about the x-axis.
    * @param yawRate the rate of change of the yaw angle.
    * @param pitchRate the rate of change of the pitch angle.
    * @param rollRate the rate of change of the roll angle.
    * @param angularVelocityToPack the angular velocity from the yaw-pitch-roll angles rate.
    *           Modified.
    */
   public static void computeAngularVelocityInBodyFrameFromYawPitchRollAnglesRate(double yaw, double pitch, double roll, double yawRate, double pitchRate,
                                                                                  double rollRate, Vector3DBasics angularVelocityToPack)
   {
      double sRoll = Math.sin(roll);
      double cRoll = Math.cos(roll);
      double sPitch = Math.sin(pitch);
      double cPitch = Math.cos(pitch);

      angularVelocityToPack.setX(rollRate - yawRate * sPitch);
      angularVelocityToPack.setY(yawRate * cPitch * sRoll + pitchRate * cRoll);
      angularVelocityToPack.setZ(yawRate * cPitch * cRoll - pitchRate * sRoll);
   }

   /**
    * Computes the angular velocity vector from the time derivatives of the yaw-pitch-roll angles.
    * <p>
    * The resulting velocity is expressed in the world coordinate system, i.e. the coordinate system
    * before the yaw-pitch-roll angles.
    * </p>
    * 
    * @param yaw the current rotation angle about the z-axis.
    * @param pitch the current rotation angle about the y-axis.
    * @param roll the current rotation angle about the x-axis.
    * @param yawRate the rate of change of the yaw angle.
    * @param pitchRate the rate of change of the pitch angle.
    * @param rollRate the rate of change of the roll angle.
    * @param angularVelocityToPack the angular velocity from the yaw-pitch-roll angles rate.
    *           Modified.
    */
   public static void computeAngularVelocityInWorldFrameFromYawPitchRollAnglesRate(double yaw, double pitch, double roll, double yawRate, double pitchRate,
                                                                                   double rollRate, Vector3DBasics angularVelocityToPack)
   {
      double sYaw = Math.sin(yaw);
      double cYaw = Math.cos(yaw);
      double sPitch = Math.sin(pitch);
      double cPitch = Math.cos(pitch);

      angularVelocityToPack.setX(rollRate * cYaw * cPitch - pitchRate * sYaw);
      angularVelocityToPack.setY(rollRate * sYaw * cPitch + pitchRate * cYaw);
      angularVelocityToPack.setZ(yawRate - rollRate * sPitch);
   }

   /**
    * Computes the time derivative of the yaw-pitch-roll angles from the angular velocity vector.
    * <p>
    * The angular velocity vector is assumed to be expressed in the local coordinate system
    * described by the yaw-pitch-roll angles.
    * </p>
    * 
    * @param angularVelocityInBody the angular velocity vector. Not modified.
    * @param yaw the current rotation angle about the z-axis.
    * @param pitch the current rotation angle about the y-axis.
    * @param roll the current rotation angle about the x-axis.
    * @param yawPitchRollRatesToPack the rate of change of, in order, the yaw, pitch, and roll
    *           angles. Modified.
    */
   public static void computeYawPitchRollAngleRatesFromAngularVelocityInBodyFrame(Vector3DReadOnly angularVelocityInBody, double yaw, double pitch, double roll,
                                                                                  double[] yawPitchRollRatesToPack)
   {
      double sRoll = Math.sin(roll);
      double cRoll = Math.cos(roll);
      double sPitch = Math.sin(pitch);
      double cPitch = Math.cos(pitch);

      double yawRate = (angularVelocityInBody.getY() * sRoll + angularVelocityInBody.getZ() * cRoll) / cPitch;
      double pitchRate = angularVelocityInBody.getY() * cRoll - angularVelocityInBody.getZ() * sRoll;
      double rollRate = angularVelocityInBody.getX() + sPitch * yawRate;

      yawPitchRollRatesToPack[0] = yawRate;
      yawPitchRollRatesToPack[1] = pitchRate;
      yawPitchRollRatesToPack[2] = rollRate;
   }

   /**
    * Computes the time derivative of the yaw-pitch-roll angles from the angular velocity vector.
    * <p>
    * The angular velocity vector is assumed to be expressed in the world coordinate system, i.e.
    * the coordinate system before the yaw-pitch-roll angles.
    * </p>
    * 
    * @param angularVelocityInWorld the angular velocity vector. Not modified.
    * @param yaw the current rotation angle about the z-axis.
    * @param pitch the current rotation angle about the y-axis.
    * @param roll the current rotation angle about the x-axis.
    * @param yawPitchRollRatesToPack the rate of change of, in order, the yaw, pitch, and roll
    *           angles. Modified.
    */
   public static void computeYawPitchRollAngleRatesFromAngularVelocityInWorldFrame(Vector3DReadOnly angularVelocityInWorld, double yaw, double pitch,
                                                                                   double roll, double[] yawPitchRollRatesToPack)
   {
      double sYaw = Math.sin(yaw);
      double cYaw = Math.cos(yaw);
      double sPitch = Math.sin(pitch);
      double cPitch = Math.cos(pitch);

      double rollRate = (angularVelocityInWorld.getX() * cYaw + angularVelocityInWorld.getY() * sYaw) / cPitch;
      double pitchRate = angularVelocityInWorld.getY() * cYaw - angularVelocityInWorld.getX() * sYaw;
      double yawRate = angularVelocityInWorld.getZ() + rollRate * sPitch;

      yawPitchRollRatesToPack[0] = yawRate;
      yawPitchRollRatesToPack[1] = pitchRate;
      yawPitchRollRatesToPack[2] = rollRate;
   }

   public static boolean quaternionEpsilonEquals(QuaternionReadOnly original, QuaternionReadOnly result, double epsilon)
   {
      if (original.epsilonEquals(result, epsilon))
      {
         return true;
      }
      else
      {
         Quaternion originalNegated = quaternionForEpsilonEquals.get();
         originalNegated.setAndNegate(original);

         return originalNegated.epsilonEquals(result, epsilon);
      }
   }

   public static void trimAxisAngleMinusPiToPi(AxisAngleReadOnly axisAngle4d, AxisAngleBasics axisAngleTrimmedToPack)
   {
      axisAngleTrimmedToPack.set(axisAngle4d);
      axisAngleTrimmedToPack.setAngle(AngleTools.trimAngleMinusPiToPi(axisAngleTrimmedToPack.getAngle()));
   }

   public static boolean axisAngleEpsilonEquals(AxisAngleReadOnly original, AxisAngleReadOnly result, double epsilon, AxisAngleComparisonMode mode)
   {
      if (mode == AxisAngleComparisonMode.IGNORE_FLIPPED_AXES)
      {
         return axisAngleEpsilonEqualsIgnoreFlippedAxes(original, result, epsilon);
      }
      else if (mode == AxisAngleComparisonMode.IGNORE_FLIPPED_AXES_ROTATION_DIRECTION_AND_COMPLETE_ROTATIONS)
      {
         AxisAngle originalMinusPiToPi = originalAxisAngleForEpsilonEquals.get();
         trimAxisAngleMinusPiToPi(original, originalMinusPiToPi);
         AxisAngle resultMinusPiToPi = resultAxisAngleForEpsilonEquals.get();
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

   public static boolean axisAngleEpsilonEqualsIgnoreFlippedAxes(AxisAngleReadOnly original, AxisAngleReadOnly result, double epsilon)
   {
      if (original.epsilonEquals(result, epsilon))
      {
         return true;
      }
      else
      {
         AxisAngle originalNegated = originalAxisAngleForEpsilonEquals.get();
         originalNegated.setAndNegate(original);

         return originalNegated.epsilonEquals(result, epsilon);
      }
   }
}
