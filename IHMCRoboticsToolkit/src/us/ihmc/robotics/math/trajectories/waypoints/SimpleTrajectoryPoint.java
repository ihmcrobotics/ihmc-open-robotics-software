package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.TrajectoryPointInterface;

public class SimpleTrajectoryPoint<W extends GeometryObject<W>, T extends SimpleTrajectoryPoint<W, T>> implements TrajectoryPointInterface<T>
{
   private double time;
   protected final W waypointData;

   public SimpleTrajectoryPoint(W waypointData)
   {
      this.waypointData = waypointData;
   }

   @Override
   public final void setTime(double time)
   {
      this.time = time;
   }

   @Override
   public final void addTimeOffset(double timeOffsetToAdd)
   {
      time += timeOffsetToAdd;
   }

   @Override
   public final void subtractTimeOffset(double timeOffsetToSubtract)
   {
      time -= timeOffsetToSubtract;
   }

   @Override
   public final void set(T other)
   {
      time = other.getTime();
      waypointData.set(other.waypointData);
   }

   @Override
   public final void setTimeToZero()
   {
      time = 0.0;
   }

   @Override
   public final void setToZero()
   {
      setTimeToZero();
      waypointData.setToZero();
   }

   @Override
   public final void setTimeToNaN()
   {
      time = Double.NaN;
   }

   @Override
   public final void setToNaN()
   {
      setTimeToNaN();
      waypointData.setToNaN();
   }

   @Override
   public final boolean containsNaN()
   {
      return Double.isNaN(time) || waypointData.containsNaN();
   }

   @Override
   public final double getTime()
   {
      return time;
   }

   public final double get(W waypointToPack)
   {
      waypointToPack.set(waypointData);
      return time;
   }

   @Override
   public final boolean epsilonEquals(T other, double epsilon)
   {
      if (!MathTools.epsilonEquals(time, other.getTime(), epsilon))
         return false;
      return waypointData.epsilonEquals(other.waypointData, epsilon);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      waypointData.applyTransform(transform);
   }
}
