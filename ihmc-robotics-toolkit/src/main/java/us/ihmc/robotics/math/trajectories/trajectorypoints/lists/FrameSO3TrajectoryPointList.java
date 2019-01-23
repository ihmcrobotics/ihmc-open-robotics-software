package us.ihmc.robotics.math.trajectories.trajectorypoints.lists;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSO3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.FrameTrajectoryPointListInterface;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.SO3TrajectoryPointInterface;

public class FrameSO3TrajectoryPointList implements FrameTrajectoryPointListInterface<FrameSO3TrajectoryPoint>
{
   private ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
   private final RecyclingArrayList<FrameSO3TrajectoryPoint> trajectoryPoints = new RecyclingArrayList<>(FrameSO3TrajectoryPoint.class);

   @Override
   public void clear()
   {
      trajectoryPoints.clear();
   }

   public void addTrajectoryPoint(double time, QuaternionReadOnly orientation, Vector3DReadOnly angularVelocity)
   {
      trajectoryPoints.add().setIncludingFrame(getReferenceFrame(), time, orientation, angularVelocity);
   }

   @Override
   public void addTrajectoryPoint(FrameSO3TrajectoryPoint trajectoryPoint)
   {
      checkReferenceFrameMatch(trajectoryPoint);
      trajectoryPoints.add().setIncludingFrame(trajectoryPoint);
   }

   public void addTrajectoryPoint(SO3TrajectoryPointInterface trajectoryPoint)
   {
      FrameSO3TrajectoryPoint newTrajectoryPoint = trajectoryPoints.add();
      newTrajectoryPoint.setToZero(getReferenceFrame());
      newTrajectoryPoint.set(trajectoryPoint);
   }

   public void setIncludingFrame(FrameSE3TrajectoryPointList trajectoryPointList)
   {
      clear(trajectoryPointList.getReferenceFrame());
      for (int i = 0; i < trajectoryPointList.getNumberOfTrajectoryPoints(); i++)
      {
         addTrajectoryPoint(trajectoryPointList.getTrajectoryPoint(i));
      }
   }

   @Override
   public FrameSO3TrajectoryPoint getTrajectoryPoint(int trajectoryPointIndex)
   {
      return trajectoryPoints.get(trajectoryPointIndex);
   }

   @Override
   public int getNumberOfTrajectoryPoints()
   {
      return trajectoryPoints.size();
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      for (int i = 0; i < trajectoryPoints.size(); i++)
      {
         trajectoryPoints.get(i).setReferenceFrame(referenceFrame);
      }
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public String toString()
   {
      return "Frame SO3 trajectory: number of frame SO3 trajectory points = " + getNumberOfTrajectoryPoints() + ".";
   }
}
