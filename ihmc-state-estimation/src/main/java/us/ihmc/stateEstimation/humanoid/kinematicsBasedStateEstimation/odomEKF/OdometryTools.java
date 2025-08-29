package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
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
}
