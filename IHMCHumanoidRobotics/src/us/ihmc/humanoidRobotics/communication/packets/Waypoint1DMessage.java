package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.IHMCRosApiMessage;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.math.trajectories.Waypoint1DInterface;

@ClassDocumentation("This class is used to build 1D trajectory messages including jointspace trajectory messages."
      + " For 3D waypoints look at EuclideanWaypointMessage (translational), SO3WaypointMessage (rotational), and SE3WaypointMessage (translational AND rotational).")
public class Waypoint1DMessage extends IHMCRosApiMessage<Waypoint1DMessage> implements Waypoint1DInterface
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

   public Waypoint1DMessage(Waypoint1DMessage waypoint1d)
   {
      time = waypoint1d.time;
      position = waypoint1d.position;
      velocity = waypoint1d.velocity;
   }

   public Waypoint1DMessage(double time, double position, double velocity)
   {
      this.time = time;
      this.position = position;
      this.velocity = velocity;
   }

   @Override
   public double getTime()
   {
      return time;
   }

   public void setTime(double time)
   {
      this.time = time;
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
      if (!MathTools.epsilonEquals(time, other.time, epsilon))
         return false;
      if (!MathTools.epsilonEquals(position, other.position, epsilon))
         return false;
      if (!MathTools.epsilonEquals(velocity, other.velocity, epsilon))
         return false;

      return true;
   }

   @Override
   public String toString()
   {
      return "SE3 waypoint: time = " + time + ", position = " + position + ", velocity = " + velocity;
   }
}
