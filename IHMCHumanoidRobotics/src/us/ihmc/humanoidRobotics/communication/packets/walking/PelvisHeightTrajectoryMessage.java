package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.humanoidRobotics.communication.packets.Waypoint1DMessage;

@ClassDocumentation("This mesage commands the controller to move the pelvis to a new height in world while going through the specified waypoints."
      + " Sending this command will not affect the pelvis horizontal position. To control the pelvis 3D position use the PelvisTrajectoryMessage instead."
      + " A third order polynomial is used to interpolate between waypoints.")
public class PelvisHeightTrajectoryMessage extends IHMCRosApiPacket<PelvisHeightTrajectoryMessage> implements VisualizablePacket
{
   @FieldDocumentation("List of waypoints to go through while executing the trajectory.")
   public Waypoint1DMessage[] waypoints;

   public PelvisHeightTrajectoryMessage()
   {
   }

   public PelvisHeightTrajectoryMessage(PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage)
   {
      waypoints = new Waypoint1DMessage[pelvisHeightTrajectoryMessage.getNumberOfWaypoints()];
      for (int i = 0; i < getNumberOfWaypoints(); i++)
         waypoints[i] = new Waypoint1DMessage(pelvisHeightTrajectoryMessage.waypoints[i]);
   }

   public PelvisHeightTrajectoryMessage(double trajectoryTime, double desiredHeight)
   {
      waypoints = new Waypoint1DMessage[] {new Waypoint1DMessage(trajectoryTime, desiredHeight, 0.0)};
   }

   public PelvisHeightTrajectoryMessage(int numberOfWaypoints)
   {
      waypoints = new Waypoint1DMessage[numberOfWaypoints];
   }

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
   public boolean epsilonEquals(PelvisHeightTrajectoryMessage other, double epsilon)
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
}
