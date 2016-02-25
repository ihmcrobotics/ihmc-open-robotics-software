package us.ihmc.robotics.math.trajectories.waypoints;

import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createName;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class YoTrajectoryPoint1D implements TrajectoryPoint1DInterface<YoTrajectoryPoint1D>
{
   private final String namePrefix;
   private final String nameSuffix;

   private final DoubleYoVariable time;
   private final DoubleYoVariable position;
   private final DoubleYoVariable velocity;

   public YoTrajectoryPoint1D(String namePrefix, String nameSuffix, YoVariableRegistry registry)
   {
      this.namePrefix = namePrefix;
      this.nameSuffix = nameSuffix;

      time = new DoubleYoVariable(createName(namePrefix, "time", nameSuffix), registry);
      position = new DoubleYoVariable(createName(namePrefix, "position", nameSuffix), registry);
      velocity = new DoubleYoVariable(createName(namePrefix, "velocity", nameSuffix), registry);
   }

   @Override
   public void setTime(double time)
   {
      this.time.set(time);
   }

   public void set(TrajectoryPoint1DInterface<?> trajectoryPoint)
   {
      time.set(trajectoryPoint.getTime());
      position.set(trajectoryPoint.getPosition());
      velocity.set(trajectoryPoint.getVelocity());
   }

   @Override
   public void set(YoTrajectoryPoint1D trajectoryPoint)
   {
      time.set(trajectoryPoint.getTime());
      position.set(trajectoryPoint.getPosition());
      velocity.set(trajectoryPoint.getVelocity());
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

   @Override
   public void addTimeOffset(double timeOffsetToAdd)
   {
      time.add(timeOffsetToAdd);
   }

   @Override
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
   public boolean epsilonEquals(YoTrajectoryPoint1D other, double epsilon)
   {
      if (!MathTools.epsilonEquals(getTime(), other.getTime(), epsilon))
         return false;
      if (!MathTools.epsilonEquals(getPosition(), other.getPosition(), epsilon))
         return false;
      if (!MathTools.epsilonEquals(getVelocity(), other.getVelocity(), epsilon))
         return false;
      return true;
   }

   @Override
   public String toString()
   {
      NumberFormat doubleFormat = new DecimalFormat(" 0.00;-0.00");
      String timeString = "time = " + doubleFormat.format(getTime());
      String positionString = "position = " + doubleFormat.format(getPosition());
      String velocityString = "velocity = " + doubleFormat.format(getVelocity());
      return "Trajectory point 1D: (" + timeString + ", " + positionString + ", " + velocityString + ")";
   }
}
