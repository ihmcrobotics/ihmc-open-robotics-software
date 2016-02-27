package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.TrajectoryPointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.TrajectoryPointListInterface;

public class SimpleTrajectoryPointList<P extends TrajectoryPointInterface<P>> implements TrajectoryPointListInterface<P, SimpleTrajectoryPointList<P>>
{
   protected final RecyclingArrayList<P> trajectoryPoints;

   public SimpleTrajectoryPointList(Class<P> trajectoryPointClass)
   {
      trajectoryPoints = new RecyclingArrayList<>(trajectoryPointClass);
   }

   @Override
   public void clear()
   {
      trajectoryPoints.clear();
   }

   @Override
   public void addTrajectoryPoint(P trajectoryPoint)
   {
      trajectoryPoints.add().set(trajectoryPoint);
   }

   @Override
   public void set(SimpleTrajectoryPointList<P> other)
   {
      clear();
      for (int i = 0; i < other.getNumberOfTrajectoryPoints(); i++)
         addTrajectoryPoint(other.getTrajectoryPoint(i));
   }

   @Override
   public int getNumberOfTrajectoryPoints()
   {
      return trajectoryPoints.size();
   }

   @Override
   public P getTrajectoryPoint(int trajectoryPointIndex)
   {
      return trajectoryPoints.get(trajectoryPointIndex);
   }

   @Override
   public P getLastTrajectoryPoint()
   {
      return trajectoryPoints.getLast();
   }

   @Override
   public double getTrajectoryTime()
   {
      return getLastTrajectoryPoint().getTime();
   }

   @Override
   public boolean epsilonEquals(SimpleTrajectoryPointList<P> other, double epsilon)
   {
      if (getNumberOfTrajectoryPoints() != other.getNumberOfTrajectoryPoints())
         return false;
      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
      {
         P thisTrajectoryPoint = getTrajectoryPoint(i);
         P otherTrajectoryPoint = other.getTrajectoryPoint(i);
         if (!thisTrajectoryPoint.epsilonEquals(otherTrajectoryPoint, epsilon))
            return false;
      }
      return true;
   }

   @Override
   public String toString()
   {
      if (trajectoryPoints != null)
         return "Simple trajectory: number of trajectory points = " + getNumberOfTrajectoryPoints() + ".";
      else
         return "Simple trajectory: no trajectory point.";
   }
}
