package us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory;

import java.util.Arrays;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationBuildOrder.ConfigurationSpaceName;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.tools.ArrayTools;

public class WaypointBasedTrajectoryMessage extends Packet<WaypointBasedTrajectoryMessage>
{
   /**
    * This is the unique hash code of the end-effector to be solved for. It used on the solver side
    * to retrieve the desired end-effector to be controlled.
    */
   public long endEffectorNameBasedHashCode;
   public double[] waypointTimes;
   public Pose3D[] waypoints;
   public ConfigurationSpaceName[] unconstrainedDegreesOfFreedom;

   public WaypointBasedTrajectoryMessage()
   {
      // empty constructor for deserialization
      setUniqueId(Packet.VALID_MESSAGE_DEFAULT_ID);
   }

   public WaypointBasedTrajectoryMessage(RigidBody endEffector, double[] waypointTimes, Pose3D[] waypoints)
   {
      this(endEffector, waypointTimes, waypoints, null);
   }

   public WaypointBasedTrajectoryMessage(RigidBody endEffector, double[] waypointTimes, Pose3D[] waypoints,
                                               ConfigurationSpaceName[] unconstrainedDegreesOfFreedom)
   {
      endEffectorNameBasedHashCode = endEffector.getNameBasedHashCode();
      setWaypoints(waypointTimes, waypoints);
      this.unconstrainedDegreesOfFreedom = unconstrainedDegreesOfFreedom;
      setUniqueId(Packet.VALID_MESSAGE_DEFAULT_ID);
   }

   public void setWaypoints(double[] waypointTimes, Pose3D[] waypoints)
   {
      if (waypointTimes.length != waypoints.length)
         throw new RuntimeException("Inconsistent array lengths.");

      this.waypointTimes = waypointTimes;
      this.waypoints = waypoints;
   }

   public void setUnconstrainedDegreesOfFreedom(ConfigurationSpaceName[] unconstrainedDegreesOfFreedom)
   {
      this.unconstrainedDegreesOfFreedom = unconstrainedDegreesOfFreedom;
   }

   public long getEndEffectorNameBasedHashCode()
   {
      return endEffectorNameBasedHashCode;
   }

   public double getWaypointTime(int i)
   {
      return waypointTimes[i];
   }

   public double[] getWaypointTimes()
   {
      return waypointTimes;
   }

   public Pose3D getWaypoint(int i)
   {
      return waypoints[i];
   }

   public Pose3D[] getWaypoints()
   {
      return waypoints;
   }

   public ConfigurationSpaceName getUnconstrainedDegreeOfFreedom(int i)
   {
      return unconstrainedDegreesOfFreedom[i];
   }

   public ConfigurationSpaceName[] getUnconstrainedDegreesOfFreedom()
   {
      return unconstrainedDegreesOfFreedom;
   }

   public int getNumberOfWaypoints()
   {
      return waypoints.length;
   }

   public int getNumberOfUnconstrainedDegreesOfFreedom()
   {
      return unconstrainedDegreesOfFreedom == null ? 0 : unconstrainedDegreesOfFreedom.length;
   }

   @Override
   public boolean epsilonEquals(WaypointBasedTrajectoryMessage other, double epsilon)
   {
      if (getNumberOfWaypoints() != other.getNumberOfWaypoints())
         return false;

      if (!ArrayTools.deltaEquals(waypointTimes, other.waypointTimes, epsilon))
         return false;

      for (int i = 0; i < getNumberOfWaypoints(); i++)
      {
         if (!waypoints[i].epsilonEquals(other.waypoints[i], epsilon))
            return false;
      }

      if (!Arrays.equals(unconstrainedDegreesOfFreedom, other.unconstrainedDegreesOfFreedom))
         return false;

      return true;
   }
}
