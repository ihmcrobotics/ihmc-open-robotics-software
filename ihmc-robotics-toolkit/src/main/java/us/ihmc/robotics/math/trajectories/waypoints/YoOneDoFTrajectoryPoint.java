package us.ihmc.robotics.math.trajectories.waypoints;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import us.ihmc.robotics.geometry.yoWaypoints.YoOneDoFWaypoint;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.OneDoFTrajectoryPointInterface;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class YoOneDoFTrajectoryPoint implements OneDoFTrajectoryPointInterface
{
   private final String namePrefix;
   private final String nameSuffix;

   private final YoOneDoFWaypoint waypoint;
   private final YoTrajectoryPoint trajectoryPoint;

   public YoOneDoFTrajectoryPoint(String namePrefix, String nameSuffix, YoVariableRegistry registry)
   {
      this.namePrefix = namePrefix;
      this.nameSuffix = nameSuffix;

      trajectoryPoint = new YoTrajectoryPoint(namePrefix, nameSuffix, registry);
      waypoint = new YoOneDoFWaypoint(namePrefix, nameSuffix, registry);
   }

   @Override
   public void setPosition(double position)
   {
      waypoint.setPosition(position);
   }

   @Override
   public void setVelocity(double velocity)
   {
      waypoint.setVelocity(velocity);
   }

   @Override
   public double getPosition()
   {
      return waypoint.getPosition();
   }

   @Override
   public double getVelocity()
   {
      return waypoint.getVelocity();
   }

   @Override
   public void setTime(double time)
   {
      trajectoryPoint.setTime(time);
   }

   @Override
   public double getTime()
   {
      return trajectoryPoint.getTime();
   }

   public String getNamePrefix()
   {
      return namePrefix;
   }

   public String getNameSuffix()
   {
      return nameSuffix;
   }

   @Override
   public String toString()
   {
      NumberFormat doubleFormat = new DecimalFormat(" 0.00;-0.00");
      String timeString = "time = " + doubleFormat.format(getTime());
      return "Trajectory point 1D: (" + timeString + ", " + waypoint + ")";
   }
}
