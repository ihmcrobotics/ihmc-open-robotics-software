package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

public interface TrajectoryPointListInterface<T extends TrajectoryPointInterface>
{
   public abstract void clear();

   public abstract void addTrajectoryPoint(T trajectoryPoint);

   public abstract T getTrajectoryPoint(int trajectoryPointIndex);

   public abstract int getNumberOfTrajectoryPoints();

   public default void set(TrajectoryPointListInterface<T> other)
   {
      clear();
      for (int i = 0; i < other.getNumberOfTrajectoryPoints(); i++)
      {
         addTrajectoryPoint(other.getTrajectoryPoint(i));
      }
   }

   public default void addTimeOffset(double timeOffsetToAdd)
   {
      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
      {
         getTrajectoryPoint(i).addTimeOffset(timeOffsetToAdd);
      }
   }

   public default void subtractTimeOffset(double timeOffsetToSubtract)
   {
      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
      {
         getTrajectoryPoint(i).subtractTimeOffset(timeOffsetToSubtract);
      }
   }

   public default T getLastTrajectoryPoint()
   {
      return getTrajectoryPoint(getNumberOfTrajectoryPoints());
   }

   public default double getTrajectoryTime()
   {
      return getLastTrajectoryPoint().getTime();
   }
}
