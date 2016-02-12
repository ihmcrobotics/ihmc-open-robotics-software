package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.IHMCRosApiMessage;
import us.ihmc.humanoidRobotics.communication.packets.Waypoint1DMessage;
import us.ihmc.robotics.math.trajectories.TrajectoryWaypoint1DDataInterface;
import us.ihmc.robotics.random.RandomTools;

@ClassDocumentation("This class is used to build trajectory messages in jointspace. It holds all the waypoints to go through with a one-dimensional trajectory."
      + " A third order polynomial function is used to interpolate between waypoints.")
public class Trajectory1DMessage extends IHMCRosApiMessage<Trajectory1DMessage> implements TrajectoryWaypoint1DDataInterface
{
   @FieldDocumentation("List of waypoints to go through while executing the trajectory.")
   public Waypoint1DMessage[] waypoints;

   /**
    * Empty constructor for serialization.
    */
   public Trajectory1DMessage()
   {
   }

   public Trajectory1DMessage(Trajectory1DMessage trajectory1dMessage)
   {
      waypoints = new Waypoint1DMessage[trajectory1dMessage.getNumberOfWaypoints()];
      for (int i = 0; i < getNumberOfWaypoints(); i++)
         waypoints[i] = new Waypoint1DMessage(trajectory1dMessage.waypoints[i]);
   }

   /**
    * Use this constructor to go straight to the given end point.
    * @param trajectoryTime how long it takes to reach the desired position.
    * @param desiredPosition desired end point position.
    */
   public Trajectory1DMessage(double trajectoryTime, double desiredPosition)
   {
      waypoints = new Waypoint1DMessage[] {new Waypoint1DMessage(trajectoryTime, desiredPosition, 0.0)};
   }

   /**
    * Use this constructor to build a message with more than one waypoint.
    * This constructor only allocates memory for the waypoints, you need to call {@link #setWaypoint(int, double, double, double)} for each waypoint afterwards.
    * @param numberOfWaypoints number of waypoints that will be sent to the controller.
    */
   public Trajectory1DMessage(int numberOfWaypoint)
   {
      waypoints = new Waypoint1DMessage[numberOfWaypoint];
   }

   /**
    * Create a waypoint.
    * @param waypointIndex index of the waypoint to create.
    * @param time time at which the waypoint has to be reached. The time is relative to when the trajectory starts.
    * @param position define the desired 1D position to be reached at this waypoint.
    * @param velocity define the desired 1D velocity to be reached at this waypoint.
    */
   public void setWaypoint(int waypointIndex, double time, double position, double velocity)
   {
      rangeCheck(waypointIndex);
      waypoints[waypointIndex] = new Waypoint1DMessage(time, position, velocity);
   }

   public int getNumberOfWaypoints()
   {
      return waypoints.length;
   }

   public Waypoint1DMessage getWaypoint(int waypointIndex)
   {
      return waypoints[waypointIndex];
   }

   public Waypoint1DMessage[] getWaypoints()
   {
      return waypoints;
   }

   private void rangeCheck(int waypointIndex)
   {
      if (waypointIndex >= getNumberOfWaypoints() || waypointIndex < 0)
         throw new IndexOutOfBoundsException("Waypoint index: " + waypointIndex + ", number of waypoints: " + getNumberOfWaypoints());
   }

   @Override
   public boolean epsilonEquals(Trajectory1DMessage other, double epsilon)
   {
      if (getNumberOfWaypoints() != other.getNumberOfWaypoints())
         return false;
      
      for (int i = 0; i < getNumberOfWaypoints(); i++)
      {
         if (!waypoints[i].epsilonEquals(other.waypoints[i], epsilon))
            return false;
      }

      return true;
   }

   public Trajectory1DMessage(Random random)
   {
      this(random.nextInt(16) + 1);

      for (int i = 0; i < getNumberOfWaypoints(); i++)
      {
         double time = RandomTools.generateRandomDoubleWithEdgeCases(random, 0.01);
         double position = RandomTools.generateRandomDoubleWithEdgeCases(random, 0.01);
         double velocity = RandomTools.generateRandomDoubleWithEdgeCases(random, 0.01);
         setWaypoint(i, time, position, velocity);
      }
   }

   @Override
   public String toString()
   {
      if (waypoints != null)
         return "Trajectory 1D: number of 1D waypoints = " + getNumberOfWaypoints();
      else
         return "Trajectory 1D: no 1D waypoints";
   }
}
