package us.ihmc.robotics.math.trajectories.waypoints;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SE3TrajectoryPointInterface;

public class FrameSE3TrajectoryPointList extends FrameTrajectoryPointList<FrameSE3TrajectoryPointList, FrameSE3TrajectoryPoint, SimpleSE3TrajectoryPoint>
{
   public FrameSE3TrajectoryPointList()
   {
      super(FrameSE3TrajectoryPoint.class);
   }

   public void addTrajectoryPoint(double time, Point3d position, Quat4d orientation, Vector3d linearVelocity, Vector3d angularVelocity)
   {
      FrameSE3TrajectoryPoint newTrajectoryPoint = addAndInitializeTrajectoryPoint();
      newTrajectoryPoint.set(time, position, orientation, linearVelocity, angularVelocity);
   }

   public void addTrajectoryPoint(SE3TrajectoryPointInterface<?> trajectoryPoint)
   {
      FrameSE3TrajectoryPoint newTrajectoryPoint = addAndInitializeTrajectoryPoint();
      newTrajectoryPoint.set(trajectoryPoint);
   }

   @Override
   public String toString()
   {
      if (trajectoryPoints != null)
         return "Frame SE3 trajectory: number of frame SE3 trajectory points = " + getNumberOfTrajectoryPoints() + ".";
      else
         return "Frame SE3 trajectory: no frame SE3 trajectory point.";
   }
}
