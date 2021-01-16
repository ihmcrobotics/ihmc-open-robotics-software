package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.Axis3D;
import us.ihmc.robotics.time.TimeIntervalBasics;
import us.ihmc.robotics.time.TimeIntervalProvider;

public interface Polynomial3DReadOnly extends FramePositionTrajectoryGenerator, TimeIntervalProvider
{
   PolynomialReadOnly getAxis(int ordinal);

   default PolynomialReadOnly getAxis(Axis3D axis)
   {
      return getAxis(axis.ordinal());
   }

   default void compute(double t)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).compute(t);
   }

   default boolean isDone()
   {
      for (int i = 0; i < 3; i++)
      {
         if (!getAxis(i).isDone())
            return false;
      }

      return true;
   }

   default double getDuration()
   {
      return getTimeInterval().getDuration();
   }

   default TimeIntervalBasics getTimeInterval()
   {
      return getAxis(0).getTimeInterval();
   }

   default boolean timeIntervalContains(double timeToCheck, double EPSILON)
   {
      return getTimeInterval().epsilonContains(timeToCheck, EPSILON);
   }

   default boolean timeIntervalContains(double timeToCheck)
   {
      return getTimeInterval().intervalContains(timeToCheck);
   }
}
