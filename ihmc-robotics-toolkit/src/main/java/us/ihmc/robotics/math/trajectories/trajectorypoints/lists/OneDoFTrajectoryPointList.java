package us.ihmc.robotics.math.trajectories.trajectorypoints.lists;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.trajectorypoints.OneDoFTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.OneDoFTrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.TrajectoryPointListBasics;

public class OneDoFTrajectoryPointList implements TrajectoryPointListBasics<OneDoFTrajectoryPoint>
{
   private final RecyclingArrayList<OneDoFTrajectoryPoint> trajectoryPoints = new RecyclingArrayList<>(OneDoFTrajectoryPoint.class);

   @Override
   public void addTrajectoryPoint(OneDoFTrajectoryPoint trajectoryPoint)
   {
      trajectoryPoints.add().set(trajectoryPoint);
   }

   public void addTrajectoryPoint(OneDoFTrajectoryPointBasics trajectoryPoint)
   {
      trajectoryPoints.add().set(trajectoryPoint);
   }

   public void addTrajectoryPoint(double time, double position, double velocity, double acceleration)
   {
      trajectoryPoints.add().set(time, position, velocity, acceleration);
   }

   @Override
   public void clear()
   {
      trajectoryPoints.clear();
   }

   @Override
   public OneDoFTrajectoryPoint getTrajectoryPoint(int trajectoryPointIndex)
   {
      return trajectoryPoints.get(trajectoryPointIndex);
   }

   @Override
   public int getNumberOfTrajectoryPoints()
   {
      return trajectoryPoints.size();
   }

   public boolean epsilonEquals(OneDoFTrajectoryPointList other, double epsilon)
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
      return "Simple trajectory 1D: number of trajectory points 1D = " + getNumberOfTrajectoryPoints() + ".";
   }
}
