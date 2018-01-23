package us.ihmc.robotics.math.trajectories.waypoints;

import java.util.List;

import us.ihmc.euclid.rotationConversion.RotationVectorConversion;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
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
         //convertQuaternionToYawPitchRoll(trajectoryPoint.getOrientation(), orientation);
         RotationVectorConversion.convertQuaternionToRotationVector(trajectoryPoint.getOrientation(), orientation);

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
}
