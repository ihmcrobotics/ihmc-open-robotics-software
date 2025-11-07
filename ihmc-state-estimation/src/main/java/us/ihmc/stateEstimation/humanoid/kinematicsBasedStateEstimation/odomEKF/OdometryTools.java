package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.matrixlib.MatrixTools;

public class OdometryTools
{
   private static final DMatrixRMaj eye3x3 = CommonOps_DDRM.identity(3);

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

   static void toSkewSymmetricMatrix(double x, double y, double z, int row, int col, DMatrixRMaj matrixToPack)
   {
      matrixToPack.zero();
      matrixToPack.set(row, col + 1, -z);
      matrixToPack.set(row, col + 2, y);
      matrixToPack.set(row + 1, col, z);
      matrixToPack.set(row + 1, col + 2, -x);
      matrixToPack.set(row + 2, col, -y);
      matrixToPack.set(row + 2, col + 1, x);
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

   static DMatrixRMaj lOperator(DMatrixRMaj quaternion)
   {
      return lOperator(quaternion.get(0), quaternion.get(1), quaternion.get(2), quaternion.get(3));
   }

   static DMatrixRMaj lOperator(QuaternionReadOnly quaternion)
   {
      return lOperator(quaternion.getS(), quaternion.getX(), quaternion.getY(), quaternion.getZ());
   }

   static DMatrixRMaj lOperator(double qs, double qx, double qy, double qz)
   {
      DMatrixRMaj lOperator = new DMatrixRMaj(4, 4);
      lOperator(qs, qx, qy, qz, lOperator);
      return lOperator;
   }

   static void lOperator(QuaternionReadOnly quaternion, DMatrixRMaj lOperatorToPack)
   {
      lOperator(quaternion.getS(), quaternion.getX(), quaternion.getY(), quaternion.getZ(), lOperatorToPack);
   }

   static void lOperator(double qs, double qx, double qy, double qz, DMatrixRMaj lOperatorToPack)
   {
      toSkewSymmetricMatrix(qx, qy, qz, 1, 1, lOperatorToPack);
      for (int i = 1; i < 4; i++)
         lOperatorToPack.add(i, i, qs);
      lOperatorToPack.set(0, 0, qs);
      lOperatorToPack.set(0, 1, -qx);
      lOperatorToPack.set(0, 2, -qy);
      lOperatorToPack.set(0, 3, -qz);
      lOperatorToPack.set(1, 0, qx);
      lOperatorToPack.set(2, 0, qy);
      lOperatorToPack.set(3, 0, qz);
   }

   static DMatrixRMaj rOperator(DMatrixRMaj quaternion)
   {
      return rOperator(quaternion.get(0), quaternion.get(1), quaternion.get(2), quaternion.get(3));
   }

   static DMatrixRMaj rOperator(QuaternionReadOnly quaternion)
   {
      return rOperator(quaternion.getS(), quaternion.getX(), quaternion.getY(), quaternion.getZ());
   }

   static DMatrixRMaj rOperator(double qs, double qx, double qy, double qz)
   {
      DMatrixRMaj rOperator = new DMatrixRMaj(4, 4);
      rOperator(qs, qx, qy, qz, rOperator);
      return rOperator;
   }

   static void rOperator(QuaternionReadOnly quaternion, DMatrixRMaj lOperatorToPack)
   {
      rOperator(quaternion.getS(), quaternion.getX(), quaternion.getY(), quaternion.getZ(), lOperatorToPack);
   }

   static void rOperator(double qs, double qx, double qy, double qz, DMatrixRMaj rOperatorToPack)
   {
      toSkewSymmetricMatrix(-qx, -qy, -qz, 1, 1, rOperatorToPack);
      for (int i = 1; i < 4; i++)
         rOperatorToPack.add(i, i, qs);
      rOperatorToPack.set(0, 0, qs);
      rOperatorToPack.set(0, 1, -qx);
      rOperatorToPack.set(0, 2, -qy);
      rOperatorToPack.set(0, 3, -qz);
      rOperatorToPack.set(1, 0, qx);
      rOperatorToPack.set(2, 0, qy);
      rOperatorToPack.set(3, 0, qz);
   }

   static void l3Operator(QuaternionReadOnly quaternion, DMatrixRMaj matrixToPack)
   {
      toSkewSymmetricMatrix(quaternion.getX(), quaternion.getY(), quaternion.getZ(), matrixToPack);
      MatrixTools.addDiagonal(matrixToPack, quaternion.getS());
   }

   static void r3Operator(QuaternionReadOnly quaternion, DMatrixRMaj matrixToPack)
   {
      toSkewSymmetricMatrix(quaternion.getX(), quaternion.getY(), quaternion.getZ(), matrixToPack);
      CommonOps_DDRM.scale(-1.0, matrixToPack);
      MatrixTools.addDiagonal(matrixToPack, quaternion.getS());
   }

   static DMatrixRMaj exponentialMap(DMatrixRMaj vector)
   {
      return exponentialMap(vector.get(0), vector.get(1), vector.get(2));
   }

   static DMatrixRMaj exponentialMap(Vector3DReadOnly vector)
   {
      return exponentialMap(vector.getX(), vector.getY(), vector.getZ());
   }

   static DMatrixRMaj exponentialMap(double x, double y, double z)
   {
      DMatrixRMaj map = new DMatrixRMaj(4, 1);

      double length = EuclidCoreTools.norm(x, y, z);

      if (Math.abs(length) < 1e-5 )
      {
         map.set(0, 0, 1.0);
         return map;
      }
      double s = Math.sin(length / 2.0);
      map.set(0, 0, Math.cos(length / 2.0));
      map.set(1, 0, s * x / length);
      map.set(2, 0, s * y / length);
      map.set(3, 0, s * z / length);

      return map;
   }

   static DMatrixRMaj logMap(QuaternionReadOnly quaternion)
   {
      return logMap(quaternion.getS(), quaternion.getX(), quaternion.getY(), quaternion.getZ());
   }

   static DMatrixRMaj logMap(DMatrixRMaj quaternion)
   {
      return logMap(quaternion.get(0), quaternion.get(1), quaternion.get(2), quaternion.get(3));
   }

   static DMatrixRMaj logMap(double qs, double qx, double qy, double qz)
   {
      double length = EuclidCoreTools.norm(qx, qy, qz);
      double multiplier = 2.0 * Math.atan2(length, qs) / length;
      DMatrixRMaj ret = new DMatrixRMaj(3, 1);
      ret.set(0, qx * multiplier);
      ret.set(1, qy * multiplier);
      ret.set(2, qz * multiplier);

      return ret;
   }
}
