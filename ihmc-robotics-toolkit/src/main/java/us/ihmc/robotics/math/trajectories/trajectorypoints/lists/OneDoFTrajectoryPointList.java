package us.ihmc.robotics.math.trajectories.trajectorypoints.lists;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.trajectorypoints.OneDoFTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.OneDoFTrajectoryPointInterface;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.TrajectoryPointListInterface;

public class OneDoFTrajectoryPointList implements TrajectoryPointListInterface<OneDoFTrajectoryPoint>
{
   private final RecyclingArrayList<OneDoFTrajectoryPoint> trajectoryPoints = new RecyclingArrayList<>(OneDoFTrajectoryPoint.class);

   @Override
   public void addTrajectoryPoint(OneDoFTrajectoryPoint trajectoryPoint)
   {
      trajectoryPoints.add().set(trajectoryPoint);
   }

   public void addTrajectoryPoint(OneDoFTrajectoryPointInterface trajectoryPoint)
   {
      trajectoryPoints.add().set(trajectoryPoint);
   }

   public void addTrajectoryPoint(double time, double position, double velocity)
   {
      trajectoryPoints.add().set(time, position, velocity);
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
