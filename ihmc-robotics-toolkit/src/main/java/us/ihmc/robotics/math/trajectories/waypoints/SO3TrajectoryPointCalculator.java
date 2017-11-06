package us.ihmc.robotics.math.trajectories.waypoints;

import java.util.List;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3TrajectoryPointInterface;

public class SO3TrajectoryPointCalculator
{
   public final RecyclingArrayList<SimpleSO3TrajectoryPoint> trajectoryPoints = new RecyclingArrayList<>(SimpleSO3TrajectoryPoint.class);

   public SO3TrajectoryPointCalculator()
   {

   }

   public void clear()
   {
      trajectoryPoints.clear();
   }

   public void appendTrajectoryPointOrientation(double time, Quaternion quaternion)
   {
      SimpleSO3TrajectoryPoint newTrajectoryPoint = trajectoryPoints.add();
      newTrajectoryPoint.setTime(time);
      newTrajectoryPoint.setOrientation(quaternion);
      newTrajectoryPoint.setAngularVelocityToNaN();
   }

   public void compute()
   {
      EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();

      int numberOfTrajectoryPoints = trajectoryPoints.size();
      for (int i = 0; i < numberOfTrajectoryPoints; i++)
      {
         SimpleSO3TrajectoryPoint trajectoryPoint = trajectoryPoints.get(i);
         Vector3D orientation = new Vector3D();
         // this conversion does not provide over 90 degree for pitch angle.
         //YawPitchRollConversion.convertQuaternionToYawPitchRoll(trajectoryPointsOrientation.get(i), orientation);
         convertQuaternionToYawPitchRoll(trajectoryPoint.getOrientation(), orientation);

         euclideanTrajectoryPointCalculator.appendTrajectoryPoint(trajectoryPoint.getTime(), new Point3D(orientation));
      }

      euclideanTrajectoryPointCalculator.computeTrajectoryPointVelocities(false);

      RecyclingArrayList<FrameEuclideanTrajectoryPoint> euclideanTrajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();

      for (int i = 0; i < numberOfTrajectoryPoints; i++)
      {
         Vector3D angularVelocityYawPitchRoll = new Vector3D();
         euclideanTrajectoryPoints.get(i).getLinearVelocity(angularVelocityYawPitchRoll);
         trajectoryPoints.get(i).setAngularVelocity(angularVelocityYawPitchRoll);
      }
   }

   public List<? extends SO3TrajectoryPointInterface<?>> getTrajectoryPoints()
   {
      return trajectoryPoints;
   }

   /**
    * temporary
    */

   private static final double EPS = 1.0e-12;

   public static final void convertQuaternionToYawPitchRoll(QuaternionReadOnly quaternion, Vector3DBasics eulerAnglesToPack)
   {
      if (quaternion.containsNaN())
      {
         eulerAnglesToPack.setToNaN();
         return;
      }

      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getS();

      double norm = quaternion.norm();
      if (norm < EPS)
      {
         eulerAnglesToPack.setToZero();
         return;
      }

      norm = 1.0 / norm;
      qx *= norm;
      qy *= norm;
      qz *= norm;
      qs *= norm;

      double pitch = computePitchFromQuaternionImpl(qx, qy, qz, qs);
      eulerAnglesToPack.setY(pitch);
      if (Double.isNaN(pitch))
      {
         eulerAnglesToPack.setToNaN();
      }
      else
      {
         eulerAnglesToPack.setZ(computeYawFromQuaternionImpl(qx, qy, qz, qs));
         eulerAnglesToPack.setX(computeRollFromQuaternionImpl(qx, qy, qz, qs));
      }
   }

   static double computeRollFromQuaternionImpl(double qx, double qy, double qz, double qs)
   {
      return Math.atan2(2.0 * (qy * qz + qx * qs), 1.0 - 2.0 * (qx * qx + qy * qy));
   }

   static double computePitchFromQuaternionImpl(double qx, double qy, double qz, double qs)
   {
      double pitchArgument = 2.0 * (qs * qy - qx * qz);

      double pitch = Math.asin(pitchArgument);
      return pitch;
   }

   static double computeYawFromQuaternionImpl(double qx, double qy, double qz, double qs)
   {
      return Math.atan2(2.0 * (qx * qy + qz * qs), 1.0 - 2.0 * (qy * qy + qz * qz));
   }
}
