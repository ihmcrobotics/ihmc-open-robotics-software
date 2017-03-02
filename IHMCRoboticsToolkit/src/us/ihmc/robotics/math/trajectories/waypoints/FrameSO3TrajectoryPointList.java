package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3TrajectoryPointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.TrajectoryPointListInterface;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameSO3TrajectoryPointList extends FrameTrajectoryPointList<FrameSO3TrajectoryPointList, FrameSO3TrajectoryPoint, SimpleSO3TrajectoryPoint>
{
   public FrameSO3TrajectoryPointList()
   {
      super(FrameSO3TrajectoryPoint.class);
   }

   public <T extends TrajectoryPointListInterface<T, ? extends SO3TrajectoryPointInterface<?>>> void setIncludingFrame(ReferenceFrame referenceFrame, T trajectoryPointList)
   {
      clear(referenceFrame);
      for (int i = 0; i < trajectoryPointList.getNumberOfTrajectoryPoints(); i++)
         addTrajectoryPoint(trajectoryPointList.getLastTrajectoryPoint());
   }
   
   public void setIncludingFrame(FrameSE3TrajectoryPointList trajectoryPointList)
   {
      clear(trajectoryPointList.getReferenceFrame());
      for (int i = 0; i < trajectoryPointList.getNumberOfTrajectoryPoints(); i++)
         addTrajectoryPoint(trajectoryPointList.getTrajectoryPoint(i));
   }

   public void addTrajectoryPoint(double time, Quaternion orientation, Vector3D angularVelocity)
   {
      FrameSO3TrajectoryPoint newTrajectoryPoint = addAndInitializeTrajectoryPoint();
      newTrajectoryPoint.set(time, orientation, angularVelocity);
   }

   public void addTrajectoryPoint(SO3TrajectoryPointInterface<?> trajectoryPoint)
   {
      FrameSO3TrajectoryPoint newTrajectoryPoint = addAndInitializeTrajectoryPoint();
      newTrajectoryPoint.set(trajectoryPoint);
   }

   @Override
   public String toString()
   {
      if (trajectoryPoints != null)
         return "Frame SO3 trajectory: number of frame SO3 trajectory points = " + getNumberOfTrajectoryPoints() + ".";
      else
         return "Frame SO3 trajectory: no frame SO3 trajectory point.";
   }
}
