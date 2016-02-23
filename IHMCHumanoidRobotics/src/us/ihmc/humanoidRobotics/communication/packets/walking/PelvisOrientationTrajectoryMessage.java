package us.ihmc.humanoidRobotics.communication.packets.walking;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.IHMCRosApiMessage;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.humanoidRobotics.communication.TransformableDataObject;
import us.ihmc.humanoidRobotics.communication.packets.SO3WaypointMessage;
import us.ihmc.robotics.geometry.RigidBodyTransform;

@ClassDocumentation("This message commands the controller to move in taskspace the pelvis to the desired orientation while going through the specified waypoints."
      + " A hermite based curve (third order) is used to interpolate the orientations."
      + " This message allows controlling the pelvis orientation without interferring with position that will still be controlled to maintain the current desired capture poit position."
      + " To excute a normal trajectory to reach a desired pelvis orientation, set only one waypoint with zero velocity and its time to be equal to the desired trajectory time."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.")
public class PelvisOrientationTrajectoryMessage extends IHMCRosApiMessage<PelvisOrientationTrajectoryMessage> implements TransformableDataObject<PelvisOrientationTrajectoryMessage>, VisualizablePacket
{
   @FieldDocumentation("List of waypoints (in taskpsace) to go through while executing the trajectory. All the information contained in these waypoints needs to be expressed in world frame.")
   public SO3WaypointMessage[] taskspaceWaypoints;

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public PelvisOrientationTrajectoryMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Clone constructor.
    * @param pelvisOrientationTrajectoryMessage message to clone.
    */
   public PelvisOrientationTrajectoryMessage(PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage)
   {
      setUniqueId(pelvisOrientationTrajectoryMessage.getUniqueId());
      setDestination(pelvisOrientationTrajectoryMessage.getDestination());
      taskspaceWaypoints = new SO3WaypointMessage[pelvisOrientationTrajectoryMessage.getNumberOfWaypoints()];
      for (int i = 0; i < getNumberOfWaypoints(); i++)
         taskspaceWaypoints[i] = new SO3WaypointMessage(pelvisOrientationTrajectoryMessage.taskspaceWaypoints[i]);
   }

   /**
    * Use this constructor to execute a simple interpolation towards the given endpoint.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredOrientation desired pelvis orientation expressed in world frame.
    */
   public PelvisOrientationTrajectoryMessage(double trajectoryTime, Quat4d desiredOrientation)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      Vector3d zeroAngularVelocity = new Vector3d();
      taskspaceWaypoints = new SO3WaypointMessage[] {new SO3WaypointMessage(trajectoryTime, desiredOrientation, zeroAngularVelocity)};
   }

   /**
    * Use this constructor to build a message with more than one waypoint.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * This constructor only allocates memory for the waypoints, you need to call either {@link #setWaypoint(int, double, Quat4d)} or {@link #setWaypoint(int, double, Quat4d, Vector3d)} for each waypoint afterwards.
    * @param numberOfWaypoints number of waypoints that will be sent to the controller.
    */
   public PelvisOrientationTrajectoryMessage(int numberOfWaypoints)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      taskspaceWaypoints = new SO3WaypointMessage[numberOfWaypoints];
   }

   /**
    * Create a waypoint.
    * @param waypointIndex index of the waypoint to create.
    * @param time time at which the waypoint has to be reached. The time is relative to when the trajectory starts.
    * @param orientation define the desired 3D orientation to be reached at this waypoint. It is expressed in world frame.
    * @param angularVelocity define the desired 3D angular velocity to be reached at this waypoint. It is expressed in world frame.
    */
   public void setWaypoint(int waypointIndex, double time, Quat4d orientation, Vector3d angularVelocity)
   {
      rangeCheck(waypointIndex);
      taskspaceWaypoints[waypointIndex] = new SO3WaypointMessage(time, orientation, angularVelocity);
   }

   public int getNumberOfWaypoints()
   {
      return taskspaceWaypoints.length;
   }

   public SO3WaypointMessage getWaypoint(int waypointIndex)
   {
      rangeCheck(waypointIndex);
      return taskspaceWaypoints[waypointIndex];
   }

   public SO3WaypointMessage[] getWaypoints()
   {
      return taskspaceWaypoints;
   }

   public SO3WaypointMessage getLastWaypoint()
   {
      return taskspaceWaypoints[getNumberOfWaypoints() - 1];
   }

   public double getTrajectoryTime()
   {
      return getLastWaypoint().time;
   }

   private void rangeCheck(int waypointIndex)
   {
      if (waypointIndex >= getNumberOfWaypoints() || waypointIndex < 0)
         throw new IndexOutOfBoundsException("Waypoint index: " + waypointIndex + ", number of waypoints: " + getNumberOfWaypoints());
   }

   @Override
   public boolean epsilonEquals(PelvisOrientationTrajectoryMessage other, double epsilon)
   {
      if (getNumberOfWaypoints() != other.getNumberOfWaypoints())
         return false;

      for (int i = 0; i < getNumberOfWaypoints(); i++)
      {
         if (!taskspaceWaypoints[i].epsilonEquals(other.taskspaceWaypoints[i], epsilon))
            return false;
      }

      return true;
   }

   @Override
   public PelvisOrientationTrajectoryMessage transform(RigidBodyTransform transform)
   {
      PelvisOrientationTrajectoryMessage transformedPelvisTrajectoryMessage = new PelvisOrientationTrajectoryMessage(getNumberOfWaypoints());

      for (int i = 0; i < getNumberOfWaypoints(); i++)
         transformedPelvisTrajectoryMessage.taskspaceWaypoints[i] = taskspaceWaypoints[i].transform(transform);

      return transformedPelvisTrajectoryMessage;
   }

   @Override
   public String toString()
   {
      if (taskspaceWaypoints != null)
         return "Pelvis SO3 trajectory: number of SO3 waypoints = " + getNumberOfWaypoints();
      else
         return "Pelvis SO3 trajectory: no SO3 waypoints";
   }
}
