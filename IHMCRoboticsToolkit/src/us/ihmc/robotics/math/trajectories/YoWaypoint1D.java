package us.ihmc.robotics.math.trajectories;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;

public class YoWaypoint1D implements Waypoint1DInterface
{
   private final String namePrefix;
   private final String nameSuffix;

   private final DoubleYoVariable time;
   private final DoubleYoVariable position;
   private final DoubleYoVariable velocity;

   public YoWaypoint1D(String namePrefix, String nameSuffix, YoVariableRegistry registry)
   {
      this.namePrefix = namePrefix;
      this.nameSuffix = nameSuffix;

      time = new DoubleYoVariable(YoFrameVariableNameTools.createName(namePrefix, "time", nameSuffix), registry);
      position = new DoubleYoVariable(YoFrameVariableNameTools.createName(namePrefix, "position", nameSuffix), registry);
      velocity = new DoubleYoVariable(YoFrameVariableNameTools.createName(namePrefix, "velocity", nameSuffix), registry);
   }

   public void set(Waypoint1DInterface waypoint1d)
   {
      time.set(waypoint1d.getTime());
      position.set(waypoint1d.getPosition());
      velocity.set(waypoint1d.getVelocity());
   }

   public void set(double time, double position, double velocity)
   {
      this.time.set(time);
      this.position.set(position);
      this.velocity.set(velocity);
   }

   public void setToNaN()
   {
      time.set(Double.NaN);
      position.set(Double.NaN);
      velocity.set(Double.NaN);
   }

   public void subtractTimeOffset(double timeOffsetToSubtract)
   {
      time.sub(timeOffsetToSubtract);
   }

   public boolean containsNaN()
   {
      return time.isNaN() || position.isNaN() || velocity.isNaN();
   }

   @Override
   public double getTime()
   {
      return time.getDoubleValue();
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
      return "(time = " + doubleFormat.format(time.getDoubleValue()) + ", position = " + doubleFormat.format(position.getDoubleValue()) + ", velocity = "
            + doubleFormat.format(velocity.getDoubleValue()) + ")";
   }
}
