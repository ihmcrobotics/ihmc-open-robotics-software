package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.OneDoFTrajectoryPointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.TrajectoryPointListInterface;

public class SimpleTrajectoryPoint1DList implements TrajectoryPointListInterface<SimpleTrajectoryPoint1D>
{
   private final RecyclingArrayList<SimpleTrajectoryPoint1D> trajectoryPoints = new RecyclingArrayList<>(SimpleTrajectoryPoint1D.class);

   @Override
   public void addTrajectoryPoint(SimpleTrajectoryPoint1D trajectoryPoint)
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
   public SimpleTrajectoryPoint1D getTrajectoryPoint(int trajectoryPointIndex)
   {
      return trajectoryPoints.get(trajectoryPointIndex);
   }

   @Override
   public int getNumberOfTrajectoryPoints()
   {
      return trajectoryPoints.size();
   }

   @Override
   public String toString()
   {
      return "Simple trajectory 1D: number of trajectory points 1D = " + getNumberOfTrajectoryPoints() + ".";
   }
}
