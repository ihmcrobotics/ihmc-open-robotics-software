package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class FrameSE3TrajectoryPointList extends FrameTrajectoryPointList<FrameSE3TrajectoryPointList, FrameSE3TrajectoryPoint, SimpleSE3TrajectoryPoint>
{
   public FrameSE3TrajectoryPointList()
   {
      super(FrameSE3TrajectoryPoint.class);
   }
   
   public void setIncludingFrame(FrameSE3TrajectoryPointList trajectoryPointList)
   {
      clear(trajectoryPointList.getReferenceFrame());
      for (int i = 0; i < trajectoryPointList.getNumberOfTrajectoryPoints(); i++)
         addTrajectoryPoint(trajectoryPointList.getTrajectoryPoint(i));
   }

   public void addTrajectoryPoint(double time, Point3D position, Quaternion orientation, Vector3D linearVelocity, Vector3D angularVelocity)
   {
      FrameSE3TrajectoryPoint newTrajectoryPoint = addAndInitializeTrajectoryPoint();
      newTrajectoryPoint.set(time, position, orientation, linearVelocity, angularVelocity);
   }

   public void addTrajectoryPoint(FrameSE3TrajectoryPoint trajectoryPoint)
   {
      FrameSE3TrajectoryPoint newTrajectoryPoint = addAndInitializeTrajectoryPoint();
      newTrajectoryPoint.set(trajectoryPoint);
   }
   
   public void addTrajectoryPoint(SimpleSE3TrajectoryPoint trajectoryPoint)
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
