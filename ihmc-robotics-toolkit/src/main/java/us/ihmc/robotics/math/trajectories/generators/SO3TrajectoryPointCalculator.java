package us.ihmc.robotics.math.trajectories.generators;

import java.util.List;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.rotationConversion.RotationVectorConversion;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.math.trajectories.trajectorypoints.SO3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.SO3TrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.FrameEuclideanTrajectoryPointList;

public class SO3TrajectoryPointCalculator
{
   public final RecyclingArrayList<SO3TrajectoryPoint> trajectoryPoints = new RecyclingArrayList<>(SO3TrajectoryPoint.class);

   public SO3TrajectoryPointCalculator()
   {

   }

   public void clear()
   {
      trajectoryPoints.clear();
   }

   public void appendTrajectoryPointOrientation(double time, Quaternion quaternion)
   {
      SO3TrajectoryPoint newTrajectoryPoint = trajectoryPoints.add();
      newTrajectoryPoint.setTime(time);
      newTrajectoryPoint.setOrientation(quaternion);
      newTrajectoryPoint.setAngularVelocityToNaN();
   }

   public void compute()
   {
      FrameEuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new FrameEuclideanTrajectoryPointCalculator();

      int numberOfTrajectoryPoints = trajectoryPoints.size();
      for (int i = 0; i < numberOfTrajectoryPoints; i++)
      {
         SO3TrajectoryPoint trajectoryPoint = trajectoryPoints.get(i);
         Vector3D orientation = new Vector3D();
         // this conversion does not provide over 90 degree for pitch angle.
         //convertQuaternionToYawPitchRoll(trajectoryPoint.getOrientation(), orientation);
         RotationVectorConversion.convertQuaternionToRotationVector(trajectoryPoint.getOrientation(), orientation);

         euclideanTrajectoryPointCalculator.appendTrajectoryPoint(trajectoryPoint.getTime(), new Point3D(orientation));
      }

      euclideanTrajectoryPointCalculator.compute(trajectoryPoints.get(numberOfTrajectoryPoints - 1).getTime());

      FrameEuclideanTrajectoryPointList euclideanTrajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();

      for (int i = 0; i < numberOfTrajectoryPoints; i++)
      {
         Vector3D angularVelocityYawPitchRoll = new Vector3D();
         euclideanTrajectoryPoints.getTrajectoryPoint(i).getLinearVelocity(angularVelocityYawPitchRoll);
         trajectoryPoints.get(i).setAngularVelocity(angularVelocityYawPitchRoll);
      }
   }

   public List<? extends SO3TrajectoryPointBasics> getTrajectoryPoints()
   {
      return trajectoryPoints;
   }
}
