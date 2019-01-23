package us.ihmc.robotics.math.trajectories.trajectorypoints.lists;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSO3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.FrameTrajectoryPointListBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.SE3TrajectoryPointBasics;

public class FrameSE3TrajectoryPointList implements FrameTrajectoryPointListBasics<FrameSE3TrajectoryPoint>
{
   private ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
   private final RecyclingArrayList<FrameSE3TrajectoryPoint> trajectoryPoints = new RecyclingArrayList<>(FrameSE3TrajectoryPoint.class);

   @Override
   public void clear()
   {
      trajectoryPoints.clear();
   }

   public void setToOrientationTrajectoryIncludingFrame(FrameSO3TrajectoryPointList trajectoryPointList)
   {
      clear(trajectoryPointList.getReferenceFrame());
      for (int i = 0; i < trajectoryPointList.getNumberOfTrajectoryPoints(); i++)
      {
         addOrientationTrajectoryPoint(trajectoryPointList.getTrajectoryPoint(i));
      }
   }

   public void addOrientationTrajectoryPoint(double time, QuaternionReadOnly orientation, Vector3DReadOnly angularVelocity)
   {
      FrameSE3TrajectoryPoint newTrajectoryPoint = trajectoryPoints.add();
      newTrajectoryPoint.setToZero(getReferenceFrame());
      newTrajectoryPoint.set(time, orientation, angularVelocity);
   }

   public void addOrientationTrajectoryPoint(FrameSO3TrajectoryPoint trajectoryPoint)
   {
      FrameSE3TrajectoryPoint newTrajectoryPoint = trajectoryPoints.add();
      newTrajectoryPoint.setToZero(getReferenceFrame());
      newTrajectoryPoint.set(trajectoryPoint);
   }

   public void setToPositionTrajectoryIncludingFrame(FrameEuclideanTrajectoryPointList trajectoryPointList)
   {
      clear(trajectoryPointList.getReferenceFrame());
      for (int i = 0; i < trajectoryPointList.getNumberOfTrajectoryPoints(); i++)
         addPositionTrajectoryPoint(trajectoryPointList.getTrajectoryPoint(i));
   }

   public void addPositionTrajectoryPoint(double time, Point3DReadOnly position, Vector3DReadOnly linearVelocity)
   {
      FrameSE3TrajectoryPoint newTrajectoryPoint = trajectoryPoints.add();
      newTrajectoryPoint.setToZero(getReferenceFrame());
      newTrajectoryPoint.set(time, position, linearVelocity);
   }

   public void addPositionTrajectoryPoint(FrameEuclideanTrajectoryPoint trajectoryPoint)
   {
      FrameSE3TrajectoryPoint newTrajectoryPoint = trajectoryPoints.add();
      newTrajectoryPoint.setToZero(getReferenceFrame());
      newTrajectoryPoint.set(trajectoryPoint);
   }

   public void addTrajectoryPoint(double time, Point3DReadOnly position, QuaternionReadOnly orientation, Vector3DReadOnly linearVelocity,
                                  Vector3DReadOnly angularVelocity)
   {
      trajectoryPoints.add().setIncludingFrame(getReferenceFrame(), time, position, orientation, linearVelocity, angularVelocity);
   }

   @Override
   public void addTrajectoryPoint(FrameSE3TrajectoryPoint trajectoryPoint)
   {
      checkReferenceFrameMatch(trajectoryPoint);
      trajectoryPoints.add().setIncludingFrame(trajectoryPoint);
   }

   public void addTrajectoryPoint(SE3TrajectoryPointBasics trajectoryPoint)
   {
      FrameSE3TrajectoryPoint newTrajectoryPoint = trajectoryPoints.add();
      newTrajectoryPoint.setToZero(getReferenceFrame());
      newTrajectoryPoint.set(trajectoryPoint);
   }

   @Override
   public FrameSE3TrajectoryPoint getTrajectoryPoint(int trajectoryPointIndex)
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
      return "Frame SE3 trajectory: number of frame SE3 trajectory points = " + getNumberOfTrajectoryPoints() + ".";
   }
}
