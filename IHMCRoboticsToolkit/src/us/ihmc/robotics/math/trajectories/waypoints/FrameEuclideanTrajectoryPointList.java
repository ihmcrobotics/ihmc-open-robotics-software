package us.ihmc.robotics.math.trajectories.waypoints;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanTrajectoryPointInterface;

public class FrameEuclideanTrajectoryPointList extends FrameTrajectoryPointList<SimpleEuclideanTrajectoryPoint, FrameEuclideanTrajectoryPoint, FrameEuclideanTrajectoryPointList>
{
   public FrameEuclideanTrajectoryPointList()
   {
      super(FrameEuclideanTrajectoryPoint.class);
   }

   public void addTrajectoryPoint(double time, Point3d position, Vector3d linearVelocity)
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
