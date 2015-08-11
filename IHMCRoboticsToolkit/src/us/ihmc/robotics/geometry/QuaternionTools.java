package us.ihmc.robotics.geometry;

import Jama.Matrix;

import java.text.DecimalFormat;
import java.text.NumberFormat;

public class QuaternionTools
{
   private static final double PI = Math.PI;
   private static final double TwoPI = 2.0 * PI;
   private static final double EPSILON = 1e-10;

// /*
//  * This will convert from quaternions to euler angles
//  * q(4,1) -> euler[phi;theta;psi] (rad)
//  */
// public static Matrix quat2euler(Matrix quat)
// {
//    double[][] q = quat.getArray();
//
//    double qs = q[0][0], qx = q[1][0], qy = q[2][0], qz = q[3][0];
//
//    double pitch = -Math.asin(2.0 * (qx * qz - qs * qy));
//    double roll, yaw;
//
//    if (Math.abs(pitch) < 0.49 * Math.PI)
//    {
//       roll = Math.atan2(2.0 * (qy * qz + qs * qx), 1.0 - 2.0 * (qx * qx + qy * qy));
//       yaw = Math.atan2(2.0 * (qx * qy + qs * qz), 1.0 - 2.0 * (qy * qy + qz * qz));
//    }
//
//    else
//    {
//       yaw = 2.0 * Math.atan2(qz, qs);
//       roll = 0.0;
//    }
//
//    double[][] m =
//        {
//        {roll},
//        {pitch},
//        {yaw}
//    };
//
//    return new Matrix(m);
// }

   public static void quaternionsToRollPitchYaw(Matrix quaternions, Matrix rollPitchYaw)
   {
      double[][] q = quaternions.getArray();

      double qs = q[0][0], qx = q[1][0], qy = q[2][0], qz = q[3][0];

      double pitch = -Math.asin(2.0 * (qx * qz - qs * qy));
      double roll, yaw;

      if (Math.abs(pitch) < 0.49 * Math.PI)
      {
         roll = Math.atan2(2.0 * (qy * qz + qs * qx), 1.0 - 2.0 * (qx * qx + qy * qy));
         yaw = Math.atan2(2.0 * (qx * qy + qs * qz), 1.0 - 2.0 * (qy * qy + qz * qz));
      }

      else
      {
         yaw = 2.0 * Math.atan2(qz, qs);
         roll = 0.0;
      }

//    double[][] m =
//        {
//        {roll},
//        {pitch},
//        {yaw}
//    };
//
//    return new Matrix(m);

      rollPitchYaw.set(0, 0, roll);
      rollPitchYaw.set(1, 0, pitch);
      rollPitchYaw.set(2, 0, yaw);
   }

// public static double[] quat2euler(double[] quat)
// {
//    double qs = quat[0], qx = quat[1], qy = quat[2], qz = quat[3];
//
//    double pitch = -Math.asin(2 * (qx * qz - qs * qy));
//    double roll, yaw;
//
//    if (Math.abs(pitch) < 0.49 * Math.PI)
//    {
//       roll = Math.atan2(2 * (qy * qz + qs * qx), 1 - 2 * (qx * qx + qy * qy));
//       yaw = Math.atan2(2 * (qx * qy + qs * qz), 1 - 2 * (qy * qy + qz * qz));
//    }
//
//    else
//    {
//       yaw = Math.atan2(qz, qs);
//       roll = 0.0;
//    }
//
//    double[] euler =
//        {
//        roll,
//        pitch,
//        yaw
//    };
//
//    return euler;
// }

   public static void quaternionsToRollPitchYaw(double[] quaternions, double[] rollPitchYaw)
   {
      double qs = quaternions[0], qx = quaternions[1], qy = quaternions[2], qz = quaternions[3];

      double pitch = -Math.asin(2 * (qx * qz - qs * qy));
      double roll, yaw;

      if (Math.abs(pitch) < 0.49 * Math.PI)
      {
         roll = Math.atan2(2 * (qy * qz + qs * qx), 1 - 2 * (qx * qx + qy * qy));
         yaw = Math.atan2(2 * (qx * qy + qs * qz), 1 - 2 * (qy * qy + qz * qz));
      }

      else
      {
         yaw = Math.atan2(qz, qs);
         roll = 0.0;
      }

//    double[] euler =
//        {
//        roll,
//        pitch,
//        yaw
//    };
//
//    return euler;

      rollPitchYaw[0] = roll;
      rollPitchYaw[1] = pitch;
      rollPitchYaw[2] = yaw;
   }

   /*
    * This will convert from euler angles to quaternion vector
    * phi, theta, psi -> q(4,1)
    * euler angles in radians in order of roll, pitch, yaw
    */
// public static Matrix euler2quat(Matrix euler)
// {
//    Transform3D transform3D = new Transform3D();
//
//    double roll = euler.get(0, 0);   // Roll
//    double pitch = euler.get(1, 0); // Pitch
//    double yaw = euler.get(2, 0);   // Yaw
//
//    transform3D.setEuler(new Vector3d(roll, pitch, yaw));
//
//    Quat4d quaternions = new Quat4d();
//    transform3D.get(quaternions);
//
//    double[][] m = new double[][]
//                   {
//                   {quaternions.w},
//                   {quaternions.x},
//                   {quaternions.y},
//                   {quaternions.z}
//    };
//
//    return new Matrix(m);
// }

// public static Matrix euler2quat(Matrix euler)
// {
//    double phi = euler.get(0, 0) / 2.0;   // Roll
//    double theta = euler.get(1, 0) / 2.0; // Pitch
//    double psi = euler.get(2, 0) / 2.0;   // Yaw
//
//    double shphi0 = Math.sin(phi);
//    double chphi0 = Math.cos(phi);
//
//    double shtheta0 = Math.sin(theta);
//    double chtheta0 = Math.cos(theta);
//
//    double shpsi0 = Math.sin(psi);
//    double chpsi0 = Math.cos(psi);
//
//    double[][] m =
//        {
//        {chphi0 * chtheta0 * chpsi0 + shphi0 * shtheta0 * shpsi0},
//        { -chphi0 * shtheta0 * shpsi0 + shphi0 * chtheta0 * chpsi0},
//        {chphi0 * shtheta0 * chpsi0 + shphi0 * chtheta0 * shpsi0},
//        {chphi0 * chtheta0 * shpsi0 - shphi0 * shtheta0 * chpsi0}
//    };
//    return new Matrix(m);
// }

   public static void rollPitchYawToQuaternions(Matrix rollPitchYaw, Matrix quaternions)
   {
      double phi = rollPitchYaw.get(0, 0) / 2.0;    // Roll
      double theta = rollPitchYaw.get(1, 0) / 2.0;    // Pitch
      double psi = rollPitchYaw.get(2, 0) / 2.0;    // Yaw

      double shphi0 = Math.sin(phi);
      double chphi0 = Math.cos(phi);

      double shtheta0 = Math.sin(theta);
      double chtheta0 = Math.cos(theta);

      double shpsi0 = Math.sin(psi);
      double chpsi0 = Math.cos(psi);

//    double[][] m =
//        {
//        {chphi0 * chtheta0 * chpsi0 + shphi0 * shtheta0 * shpsi0},
//        { -chphi0 * shtheta0 * shpsi0 + shphi0 * chtheta0 * chpsi0},
//        {chphi0 * shtheta0 * chpsi0 + shphi0 * chtheta0 * shpsi0},
//        {chphi0 * chtheta0 * shpsi0 - shphi0 * shtheta0 * chpsi0}
//    };
//    return new Matrix(m);

      quaternions.set(0, 0, chphi0 * chtheta0 * chpsi0 + shphi0 * shtheta0 * shpsi0);
      quaternions.set(1, 0, -chphi0 * shtheta0 * shpsi0 + shphi0 * chtheta0 * chpsi0);
      quaternions.set(2, 0, chphi0 * shtheta0 * chpsi0 + shphi0 * chtheta0 * shpsi0);
      quaternions.set(3, 0, chphi0 * chtheta0 * shpsi0 - shphi0 * shtheta0 * chpsi0);
   }

// public static double[] euler2quat(double[] euler)
// {
//    double phi = euler[0] / 2.0;   // Roll
//    double theta = euler[1] / 2.0; // Pitch
//    double psi = euler[2] / 2.0;   // Yaw
//
//    double shphi0 = Math.sin(phi);
//    double chphi0 = Math.cos(phi);
//
//    double shtheta0 = Math.sin(theta);
//    double chtheta0 = Math.cos(theta);
//
//    double shpsi0 = Math.sin(psi);
//    double chpsi0 = Math.cos(psi);
//
//    double[] quaternions =
//        {chphi0 * chtheta0 * chpsi0 + shphi0 * shtheta0 * shpsi0,
//         -chphi0 * shtheta0 * shpsi0 + shphi0 * chtheta0 * chpsi0,
//        chphi0 * shtheta0 * chpsi0 + shphi0 * chtheta0 * shpsi0,
//        chphi0 * chtheta0 * shpsi0 - shphi0 * shtheta0 * chpsi0};
//
//    return quaternions;
// }

   public static void rollPitchYawToQuaternions(double[] rollPitchYaw, double[] quaternions)
   {
      double phi = rollPitchYaw[0] / 2.0;    // Roll
      double theta = rollPitchYaw[1] / 2.0;    // Pitch
      double psi = rollPitchYaw[2] / 2.0;    // Yaw

      double shphi0 = Math.sin(phi);
      double chphi0 = Math.cos(phi);

      double shtheta0 = Math.sin(theta);
      double chtheta0 = Math.cos(theta);

      double shpsi0 = Math.sin(psi);
      double chpsi0 = Math.cos(psi);

//    double[] quaternions =
//        {chphi0 * chtheta0 * chpsi0 + shphi0 * shtheta0 * shpsi0,
//         -chphi0 * shtheta0 * shpsi0 + shphi0 * chtheta0 * chpsi0,
//        chphi0 * shtheta0 * chpsi0 + shphi0 * chtheta0 * shpsi0,
//        chphi0 * chtheta0 * shpsi0 - shphi0 * shtheta0 * chpsi0};
//
//    return quaternions;

      quaternions[0] = chphi0 * chtheta0 * chpsi0 + shphi0 * shtheta0 * shpsi0;
      quaternions[1] = -chphi0 * shtheta0 * shpsi0 + shphi0 * chtheta0 * chpsi0;
      quaternions[2] = chphi0 * shtheta0 * chpsi0 + shphi0 * chtheta0 * shpsi0;
      quaternions[3] = chphi0 * chtheta0 * shpsi0 - shphi0 * shtheta0 * chpsi0;
   }

   /**
    * computeAngleDifferenceMinusPiToPi: returns (angleA - angleB), where the return value is [-pi, pi)
    *
    * @param angleA double
    * @param angleB double
    * @return double
    */
   public static double computeAngleDifferenceMinusPiToPi(double angleA, double angleB)
   {
      double difference = angleA - angleB;
      difference = difference % TwoPI;
      difference = shiftAngleToStartOfRange(difference, -PI);

      return difference;
   }

   /**
    * This will shift an angle to be in the range [<i>startOfAngleRange</i>,
    *  (<i>startOfAngleRange + 2*pi</i>)
    *
    * @param angleToShift the angle to shift
    * @param startOfAngleRange start of the range.
    * @return the shifted angle
    */
   public static double shiftAngleToStartOfRange(double angleToShift, double startOfAngleRange)
   {
      double ret = angleToShift;
      startOfAngleRange = startOfAngleRange - EPSILON;

      if (angleToShift < startOfAngleRange)
      {
         ret = angleToShift + Math.ceil((startOfAngleRange - angleToShift) / (2.0 * Math.PI)) * Math.PI * 2.0;
      }

      if (angleToShift >= (startOfAngleRange + Math.PI * 2.0))
      {
         ret = angleToShift - Math.floor((angleToShift - startOfAngleRange) / (2.0 * Math.PI)) * Math.PI * 2.0;
      }

      return ret;
   }

   // Format the number to exactly 4 decimal places (or 1 less when negative).  by Shervin 7/17/07.
   public static String format4(double d)
   {
      String s;
      if (d >= 0.0)
         s = "0000";    // use exactly (N) decimal places.
      else
         s = "000";    // use exactly (N-1) decimal places after '-'.
      NumberFormat formatter = new DecimalFormat("0." + s);    // use exactly n decimal places

      return formatter.format(d);
   }
}
