package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.TrajectoryPointInterface;

public abstract class TrajectoryPoint<W extends GeometryObject<W>, T extends TrajectoryPoint<W, T>> implements TrajectoryPointInterface<T>
{
   private double time;
   final W waypointData;

   public TrajectoryPoint(W waypointData)
   {
      this.waypointData = waypointData;
   }

   @Override
   public void set(T other)
   {
      time = other.getTime();
      waypointData.set(other.waypointData);
   }

   @Override
   public void setTime(double time)
   {
      this.time = time;
   }

   @Override
   public void addTimeOffset(double timeOffsetToAdd)
   {
      time += timeOffsetToAdd;
   }

   @Override
   public void subtractTimeOffset(double timeOffsetToSubtract)
   {
      time -= timeOffsetToSubtract;
   }

   @Override
   public void setToZero()
   {
      time = 0.0;
      waypointData.setToZero();
   }

   @Override
   public void setToNaN()
   {
      time = Double.NaN;
      waypointData.setToNaN();
   }

   @Override
   public boolean containsNaN()
   {
      return Double.isNaN(time) || waypointData.containsNaN();
   }

   @Override
   public double getTime()
   {
      return time;
   }

   @Override
   public boolean epsilonEquals(T other, double epsilon)
   {
      if (!MathTools.epsilonEquals(time, other.getTime(), epsilon))
         return false;
      if (!waypointData.epsilonEquals(other.waypointData, epsilon))
         return false;
      return true;
   }
}
