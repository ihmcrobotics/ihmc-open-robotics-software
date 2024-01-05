package us.ihmc.robotics.math.trajectories.waypoints;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import us.ihmc.robotics.math.trajectories.waypoints.interfaces.OneDoFWaypointBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoOneDoFWaypoint implements OneDoFWaypointBasics
{
   private final String namePrefix;
   private final String nameSuffix;

   private final YoDouble position;
   private final YoDouble velocity;
   private final YoDouble acceleration;

   public YoOneDoFWaypoint(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      this.namePrefix = namePrefix;
      this.nameSuffix = nameSuffix;

      position = new YoDouble(YoGeometryNameTools.assembleName(namePrefix, "position", nameSuffix), registry);
      velocity = new YoDouble(YoGeometryNameTools.assembleName(namePrefix, "velocity", nameSuffix), registry);
      acceleration = new YoDouble(YoGeometryNameTools.assembleName(namePrefix, "acceleration", nameSuffix), registry);
   }

   @Override
   public void setPosition(double position)
   {
      this.position.set(position);
   }

   @Override
   public void setVelocity(double velocity)
   {
      this.velocity.set(velocity);
   }

   @Override
   public void setAcceleration(double acceleration)
   {
      this.acceleration.set(acceleration);
   }

   @Override
   public double getPosition()
   {
      return position.getDoubleValue();
   }

   @Override
   public double getVelocity()
   {
      return velocity.getDoubleValue();
   }

   @Override
   public double getAcceleration()
   {
      return acceleration.getDoubleValue();
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
      String positionString = "position = " + doubleFormat.format(getPosition());
      String velocityString = "velocity = " + doubleFormat.format(getVelocity());
      return "Waypoint 1D: (" + positionString + ", " + velocityString + ")";
   }
}
