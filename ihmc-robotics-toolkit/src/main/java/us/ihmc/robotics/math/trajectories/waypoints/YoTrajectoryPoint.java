package us.ihmc.robotics.math.trajectories.waypoints;

import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createName;

import us.ihmc.robotics.math.trajectories.waypoints.interfaces.TrajectoryPointInterface;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoTrajectoryPoint implements TrajectoryPointInterface
{
   private YoDouble time;

   public YoTrajectoryPoint(String namePrefix, String nameSuffix, YoVariableRegistry registry)
   {
      time = new YoDouble(createName(namePrefix, "time", nameSuffix), registry);
   }

   @Override
   public void setTime(double time)
   {
      this.time.set(time);
   }

   @Override
   public double getTime()
   {
      return time.getValue();
   }
}
