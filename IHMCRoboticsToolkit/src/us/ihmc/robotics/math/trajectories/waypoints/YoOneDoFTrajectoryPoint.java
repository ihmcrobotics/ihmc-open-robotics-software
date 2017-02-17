package us.ihmc.robotics.math.trajectories.waypoints;

import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createName;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.OneDoFTrajectoryPointInterface;

public class YoOneDoFTrajectoryPoint implements OneDoFTrajectoryPointInterface<YoOneDoFTrajectoryPoint>
{
   private final String namePrefix;
   private final String nameSuffix;

   private final DoubleYoVariable time;
   private final YoOneDoFWaypoint waypoint1d;

   public YoOneDoFTrajectoryPoint(String namePrefix, String nameSuffix, YoVariableRegistry registry)
   {
      this.namePrefix = namePrefix;
      this.nameSuffix = nameSuffix;

      time = new DoubleYoVariable(createName(namePrefix, "time", nameSuffix), registry);
      waypoint1d = new YoOneDoFWaypoint(namePrefix, nameSuffix, registry);
   }

   @Override
   public void setTime(double time)
   {
      this.time.set(time);
   }

   @Override
   public void setPosition(double position)
   {
      waypoint1d.setPosition(position);
   }

   @Override
   public void setVelocity(double velocity)
   {
      waypoint1d.setVelocity(velocity);
   }

   public void set(OneDoFTrajectoryPointInterface<?> trajectoryPoint)
   {
      time.set(trajectoryPoint.getTime());
      waypoint1d.set(trajectoryPoint.getPosition(), trajectoryPoint.getVelocity());
   }

   @Override
   public void set(YoOneDoFTrajectoryPoint other)
   {
      time.set(other.getTime());
      waypoint1d.set(other.waypoint1d);
   }

   public void set(double time, double position, double velocity)
   {
      this.time.set(time);
      waypoint1d.set(position, velocity);
   }

   @Override
   public void setTimeToZero()
   {
      time.set(0.0);
   }

   @Override
   public void setToZero()
   {
      setTimeToZero();
      waypoint1d.setToZero();
   }

   @Override
   public void setTimeToNaN()
   {
      time.set(Double.NaN);
   }

   @Override
   public void setToNaN()
   {
      setTimeToNaN();
      waypoint1d.setToNaN();
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

   @Override
   public boolean containsNaN()
   {
      return time.isNaN() || waypoint1d.containsNaN();
   }

   @Override
   public double getTime()
   {
      return time.getDoubleValue();
   }

   @Override
   public double getPosition()
   {
      return waypoint1d.getPosition();
   }

   @Override
   public double getVelocity()
   {
      return waypoint1d.getVelocity();
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
   public boolean epsilonEquals(YoOneDoFTrajectoryPoint other, double epsilon)
   {
      if (!MathTools.epsilonEquals(getTime(), other.getTime(), epsilon))
         return false;
      if (!waypoint1d.epsilonEquals(other.waypoint1d, epsilon))
         return false;
      return true;
   }

   @Override
   public String toString()
   {
      NumberFormat doubleFormat = new DecimalFormat(" 0.00;-0.00");
      String timeString = "time = " + doubleFormat.format(getTime());
      return "Trajectory point 1D: (" + timeString + ", " + waypoint1d + ")";
   }

   @Override
   public void applyTransform(Transform transform)
   {
      waypoint1d.applyTransform(transform);
   }
}
