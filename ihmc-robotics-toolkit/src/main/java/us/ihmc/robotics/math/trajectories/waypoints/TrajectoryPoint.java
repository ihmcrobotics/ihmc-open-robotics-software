package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.robotics.math.trajectories.waypoints.interfaces.TrajectoryPointInterface;

public class TrajectoryPoint implements TrajectoryPointInterface
{
   private double time;

   @Override
   public void setTime(double time)
   {
      this.time = time;
   }

   @Override
   public double getTime()
   {
      return time;
   }
}
