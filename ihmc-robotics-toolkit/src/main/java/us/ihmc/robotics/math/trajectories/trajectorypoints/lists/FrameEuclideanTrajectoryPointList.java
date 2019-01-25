package us.ihmc.robotics.math.trajectories.trajectorypoints.lists;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.EuclideanTrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.FrameTrajectoryPointListBasics;

public class FrameEuclideanTrajectoryPointList implements FrameTrajectoryPointListBasics<FrameEuclideanTrajectoryPoint>
{
   private ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
   private final RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = new RecyclingArrayList<>(FrameEuclideanTrajectoryPoint.class);

   @Override
   public void clear()
   {
      trajectoryPoints.clear();
   }

   public void addTrajectoryPoint(double time, Point3DReadOnly position, Vector3DReadOnly linearVelocity)
   {
      trajectoryPoints.add().setIncludingFrame(getReferenceFrame(), time, position, linearVelocity);
   }

   @Override
   public void addTrajectoryPoint(FrameEuclideanTrajectoryPoint trajectoryPoint)
   {
      checkReferenceFrameMatch(trajectoryPoint);
      trajectoryPoints.add().setIncludingFrame(trajectoryPoint);
   }

   public void addTrajectoryPoint(EuclideanTrajectoryPointBasics trajectoryPoint)
   {
      FrameEuclideanTrajectoryPoint newTrajectoryPoint = trajectoryPoints.add();
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
   public FrameEuclideanTrajectoryPoint getTrajectoryPoint(int trajectoryPointIndex)
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
      return "Frame Euclidean trajectory: number of frame Euclidean trajectory points = " + getNumberOfTrajectoryPoints() + ".";
   }
}
