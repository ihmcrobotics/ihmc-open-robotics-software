package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanTrajectoryPointInterface;

public class FrameEuclideanTrajectoryPointList extends FrameTrajectoryPointList<FrameEuclideanTrajectoryPointList, FrameEuclideanTrajectoryPoint, SimpleEuclideanTrajectoryPoint>
{
   public FrameEuclideanTrajectoryPointList()
   {
      super(FrameEuclideanTrajectoryPoint.class);
   }

   public void addTrajectoryPoint(double time, Point3D position, Vector3D linearVelocity)
   {
      FrameEuclideanTrajectoryPoint newTrajectoryPoint = addAndInitializeTrajectoryPoint();
      newTrajectoryPoint.set(time, position, linearVelocity);
   }

   public void addTrajectoryPoint(EuclideanTrajectoryPointInterface<?> trajectoryPoint)
   {
      FrameEuclideanTrajectoryPoint newTrajectoryPoint = addAndInitializeTrajectoryPoint();
      newTrajectoryPoint.set(trajectoryPoint);
   }

   @Override
   public String toString()
   {
      if (trajectoryPoints != null)
         return "Frame euclidean trajectory: number of frame euclidean trajectory points = " + getNumberOfTrajectoryPoints() + ".";
      else
         return "Frame euclidean trajectory: no frame euclidean trajectory point.";
   }
}
