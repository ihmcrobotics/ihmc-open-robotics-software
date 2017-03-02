package us.ihmc.robotics.math.trajectories.waypoints;

import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createName;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.interfaces.OneDoFWaypointInterface;

public class YoOneDoFWaypoint implements OneDoFWaypointInterface<YoOneDoFWaypoint>
{
   private final String namePrefix;
   private final String nameSuffix;

   private final DoubleYoVariable position;
   private final DoubleYoVariable velocity;

   public YoOneDoFWaypoint(String namePrefix, String nameSuffix, YoVariableRegistry registry)
   {
      this.namePrefix = namePrefix;
      this.nameSuffix = nameSuffix;

      position = new DoubleYoVariable(createName(namePrefix, "position", nameSuffix), registry);
      velocity = new DoubleYoVariable(createName(namePrefix, "velocity", nameSuffix), registry);
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

   public void set(double position, double velocity)
   {
      setPosition(position);
      setVelocity(velocity);
   }

   public void set(OneDoFWaypointInterface<?> waypoint)
   {
      position.set(waypoint.getPosition());
      velocity.set(waypoint.getVelocity());
   }

   @Override
   public void set(YoOneDoFWaypoint waypoint)
   {
      position.set(waypoint.getPosition());
      velocity.set(waypoint.getVelocity());
   }

   public void set(double time, double position, double velocity)
   {
      this.position.set(position);
      this.velocity.set(velocity);
   }

   @Override
   public void setToZero()
   {
      position.set(0.0);
      velocity.set(0.0);
   }

   @Override
   public void setToNaN()
   {
      position.set(Double.NaN);
      velocity.set(Double.NaN);
   }

   @Override
   public boolean containsNaN()
   {
      return position.isNaN() || velocity.isNaN();
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
   public boolean epsilonEquals(YoOneDoFWaypoint other, double epsilon)
   {
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
      String positionString = "position = " + doubleFormat.format(getPosition());
      String velocityString = "velocity = " + doubleFormat.format(getVelocity());
      return "Waypoint 1D: (" + positionString + ", " + velocityString + ")";
   }

   @Override
   public void applyTransform(Transform transform)
   {
      // Do nothing since simple numbers here.
   }
}
