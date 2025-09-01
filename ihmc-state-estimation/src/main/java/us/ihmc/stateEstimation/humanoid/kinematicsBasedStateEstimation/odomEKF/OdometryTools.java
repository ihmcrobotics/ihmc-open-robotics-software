package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class OdometryTools
{
   static void toRotationMatrixInverse(double qx, double qy, double qz, double qs, DMatrixRMaj rotationMatrix)
   {
      double qx2 = qx * qx;
      double qy2 = qy * qy;
      double qz2 = qz * qz;
      double qs2 = qs * qs;

      rotationMatrix.set(0, 0, qx2 - qy2 - qz2 + qs2);
      rotationMatrix.set(1, 1, -qx2 + qy2 - qz2 + qs2);
      rotationMatrix.set(2, 2, -qx2 - qy2 + qz2 + qs2);

      double xy = qx * qy;
      double xz = qx * qz;
      double yz = qy * qz;
      double xw = qx * qs;
      double yw = qy * qs;
      double zw = qz * qs;

      rotationMatrix.set(1, 0, 2.0 * (xy - zw));
      rotationMatrix.set(0, 1, 2.0 * (xy + zw));

      rotationMatrix.set(2, 0, 2.0 * (xz + yw));
      rotationMatrix.set(0, 2, 2.0 * (xz - yw));

      rotationMatrix.set(2, 1, 2.0 * (yz - xw));
      rotationMatrix.set(1, 2, 2.0 * (yz + xw));
   }

   static void toRotationMatrixInverse(QuaternionReadOnly quaternion, DMatrixRMaj rotationMatrix)
   {
      toRotationMatrixInverse(quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getS(), rotationMatrix);
   }

   static void toSkewSymmetricMatrix(double x, double y, double z, DMatrixRMaj matrixToPack)
   {
      matrixToPack.zero();
      matrixToPack.set(0, 1, -z);
      matrixToPack.set(0, 2, y);
      matrixToPack.set(1, 0, z);
      matrixToPack.set(1, 2, -x);
      matrixToPack.set(2, 0, -y);
      matrixToPack.set(2, 1, x);
   }

   static void toSkewSymmetricMatrix(Tuple3DReadOnly vector, DMatrixRMaj matrixToPack)
   {
      toSkewSymmetricMatrix(vector.getX(), vector.getY(), vector.getZ(), matrixToPack);
   }

   static void toRotationMatrix(double qx, double qy, double qz, double qs, DMatrixRMaj rotationMatrix)
   {
      double qx2 = qx * qx;
      double qy2 = qy * qy;
      double qz2 = qz * qz;
      double qs2 = qs * qs;

      rotationMatrix.set(0, 0, qx2 - qy2 - qz2 + qs2);
      rotationMatrix.set(1, 1, -qx2 + qy2 - qz2 + qs2);
      rotationMatrix.set(2, 2, -qx2 - qy2 + qz2 + qs2);

      double xy = qx * qy;
      double xz = qx * qz;
      double yz = qy * qz;
      double xw = qx * qs;
      double yw = qy * qs;
      double zw = qz * qs;

      rotationMatrix.set(0, 1, 2.0 * (xy - zw));
      rotationMatrix.set(1, 0, 2.0 * (xy + zw));

      rotationMatrix.set(0, 2, 2.0 * (xz + yw));
      rotationMatrix.set(2, 0, 2.0 * (xz - yw));

      rotationMatrix.set(1, 2, 2.0 * (yz - xw));
      rotationMatrix.set(2, 1, 2.0 * (yz + xw));
   }

   static void toRotationMatrix(QuaternionReadOnly quaternion, DMatrixRMaj rotationMatrix)
   {
      toRotationMatrix(quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getS(), rotationMatrix);
   }

   static void setDiagonals(int start, int elements, double value, DMatrixRMaj matrix)
   {
      for (int i = start; i < start + elements; i++)
         matrix.set(i, i, value);
   }

   public static void toQuaternionFromRotationVector(Vector3DReadOnly rotation, QuaternionBasics quaternionToPack)
   {
      double magnitude = rotation.norm();
      if (magnitude < 1e-7)
      {
         quaternionToPack.setToZero();
         return;
      }

      double s = Math.sin(magnitude * 0.5);
      double qs = Math.cos(magnitude * 0.5);
      double qx = s * rotation.getX() / magnitude;
      double qy = s * rotation.getY() / magnitude;
      double qz = s * rotation.getZ() / magnitude;
      quaternionToPack.set(qx, qy, qz, qs);
   }

   public static void toQuaternionFromRotationVectorSmallAngle(Vector3DReadOnly rotation, QuaternionBasics quaternionToPack)
   {
      double magnitude = rotation.norm();
      double s = magnitude * 0.5;
      double qs = 1.0;
      double qx = s * rotation.getX() / magnitude;
      double qy = s * rotation.getY() / magnitude;
      double qz = s * rotation.getZ() / magnitude;
      quaternionToPack.set(qx, qy, qz, qs);
   }

   public static void logMap(QuaternionReadOnly quaternion, Vector3DBasics rotationToPack)
   {
      // TODO is this just a transform to the axis-angle representation?
      double norm = EuclidCoreTools.norm(quaternion.getX(), quaternion.getY(), quaternion.getZ());
      double scale = 2.0 * Math.atan2(norm, quaternion.getS()) / norm;
      rotationToPack.set(quaternion.getX(), quaternion.getY(), quaternion.getZ());
      rotationToPack.scale(scale);
   }

   public static void logMapSmallAngle(QuaternionReadOnly quaternion, Vector3DBasics rotationToPack)
   {
      rotationToPack.set(quaternion.getX(), quaternion.getY(), quaternion.getZ());
      rotationToPack.scale(2.0);
   }
}
