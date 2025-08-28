package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class OdometryIndexHelper
{
   public static int getStateSizePerLink()
   {
      return 16;
   }

   public static int getStatePositionIndex()
   {
      return 0;
   }

   public static int getStateVelocityIndex()
   {
      return 3;
   }

   public static int getStateOrientationIndex()
   {
      return 6;
   }

   public static int getStateAccelerationBiasIndex()
   {
      return 10;
   }

   public static int getStateGyroBiasIndex()
   {
      return 13;
   }

   public static int getBasePositionIndex()
   {
      return getStatePositionIndex();
   }

   public static int getBaseVelocityIndex()
   {
      return getStateVelocityIndex();
   }

   public static int getBaseOrientationIndex()
   {
      return getStateOrientationIndex();
   }

   public static int getBaseAccelerationBiasIndex()
   {
      return getStateAccelerationBiasIndex();
   }

   public static int getBaseGyroBiasIndex()
   {
      return getStateGyroBiasIndex();
   }

   public static int getFootPositionIndex(int footNumber)
   {
      return getStateSizePerLink() * (footNumber + 1) + getStatePositionIndex();
   }

   public static int getFootVelocityIndex(int footNumber)
   {
      return getFootPositionIndex(footNumber) + getStateVelocityIndex();
   }

   public static int getFootOrientationIndex(int footNumber)
   {
      return getFootPositionIndex(footNumber) + getStateOrientationIndex();
   }

   public static int getFootAccelerationBiasIndex(int footNumber)
   {
      return getFootPositionIndex(footNumber) + getStateAccelerationBiasIndex();
   }

   public static int getFootGyroBiasIndex(int footNumber)
   {
      return getFootPositionIndex(footNumber) + getStateGyroBiasIndex();
   }

   public static void toQuaternionFromRotationVector(Vector3DReadOnly rotation, QuaternionBasics quaternionToPack)
   {
      double magnitude = rotation.norm();
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
