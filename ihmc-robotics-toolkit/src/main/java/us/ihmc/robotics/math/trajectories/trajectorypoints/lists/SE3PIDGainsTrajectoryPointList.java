package us.ihmc.robotics.math.trajectories.trajectorypoints.lists;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.robotics.controllers.pidGains.PID3DGains;
import us.ihmc.robotics.math.trajectories.trajectorypoints.SE3PIDGainsTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.SE3PIDGainsTrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.TrajectoryPointListBasics;

public class SE3PIDGainsTrajectoryPointList implements TrajectoryPointListBasics<SE3PIDGainsTrajectoryPoint>
{
   private final RecyclingArrayList<SE3PIDGainsTrajectoryPoint> trajectoryPoints = new RecyclingArrayList<>(SE3PIDGainsTrajectoryPoint.class);

   @Override
   public void addTrajectoryPoint(SE3PIDGainsTrajectoryPoint trajectoryPoint)
   {
      trajectoryPoints.add().set(trajectoryPoint);
   }

   public void addTrajectoryPoint(SE3PIDGainsTrajectoryPointBasics trajectoryPoint)
   {
      trajectoryPoints.add().set(trajectoryPoint);
   }

   public void addTrajectoryPoint(double time, PID3DGains angularGains, PID3DGains linearGains)
   {
      trajectoryPoints.add().set(time, angularGains, linearGains);
   }

   @Override
   public void clear()
   {
      trajectoryPoints.clear();
   }

   @Override
   public SE3PIDGainsTrajectoryPoint getTrajectoryPoint(int trajectoryPointIndex)
   {
      return trajectoryPoints.get(trajectoryPointIndex);
   }

   @Override
   public int getNumberOfTrajectoryPoints()
   {
      return trajectoryPoints.size();
   }

   public boolean epsilonEquals(SE3PIDGainsTrajectoryPointList other, double epsilon)
   {
      if (getNumberOfTrajectoryPoints() != other.getNumberOfTrajectoryPoints())
      {
         return false;
      }
      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
      {
         if (!getTrajectoryPoint(i).epsilonEquals(other.getTrajectoryPoint(i), epsilon))
         {
            return false;
         }
      }
      return true;
   }

   @Override
   public String toString()
   {
      return trajectoryPoints.toString();
   }
}
