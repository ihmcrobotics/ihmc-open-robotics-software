package us.ihmc.humanoidRobotics.communication.packets;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.IHMCRosApiMessage;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.math.trajectories.waypoints.Waypoint1DInterface;

@ClassDocumentation("This class is used to build 1D trajectory messages including jointspace trajectory messages."
      + " For 3D waypoints look at EuclideanWaypointMessage (translational), SO3WaypointMessage (rotational), and SE3WaypointMessage (translational AND rotational).")
public class Waypoint1DMessage extends IHMCRosApiMessage<Waypoint1DMessage> implements Waypoint1DInterface<Waypoint1DMessage>
{
   @FieldDocumentation("Time at which the waypoint has to be reached. The time is relative to when the trajectory starts.")
   public double time;
   @FieldDocumentation("Define the desired 1D position to be reached at this waypoint.")
   public double position;
   @FieldDocumentation("Define the desired 1D velocity to be reached at this waypoint.")
   public double velocity;

   /**
    * Empty constructor for serialization.
    */
   public Waypoint1DMessage()
   {
   }

   public Waypoint1DMessage(Waypoint1DInterface<?> waypoint1d)
   {
      time = waypoint1d.getTime();
      position = waypoint1d.getPosition();
      velocity = waypoint1d.getVelocity();
   }

   public Waypoint1DMessage(double time, double position, double velocity)
   {
      this.time = time;
      this.position = position;
      this.velocity = velocity;
   }

   @Override
   public void set(Waypoint1DMessage waypoint)
   {
      time = waypoint.time;
      position = waypoint.position;
      velocity = waypoint.velocity;
   }

   @Override
   public void setTime(double time)
   {
      this.time = time;
   }

   @Override
   public void addTimeOffset(double timeOffsetToAdd)
   {
      time += timeOffsetToAdd;
   }

   @Override
   public void subtractTimeOffset(double timeOffsetToSubtract)
   {
      time -= timeOffsetToSubtract;
   }

   @Override
   public double getTime()
   {
      return time;
   }

   @Override
   public double getPosition()
   {
      return position;
   }

   public void setPosition(double position)
   {
      this.position = position;
   }

   @Override
   public double getVelocity()
   {
      return velocity;
   }

   public void setVelocity(double velocity)
   {
      this.velocity = velocity;
   }

   @Override
   public boolean epsilonEquals(Waypoint1DMessage other, double epsilon)
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
      return "(" + timeString + ", " + positionString + ", " + velocityString + ")";
   }
}
