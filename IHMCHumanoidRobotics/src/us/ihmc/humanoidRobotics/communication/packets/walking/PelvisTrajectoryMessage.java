package us.ihmc.humanoidRobotics.communication.packets.walking;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.humanoidRobotics.communication.TransformableDataObject;
import us.ihmc.humanoidRobotics.communication.packets.SE3WaypointMessage;
import us.ihmc.robotics.geometry.RigidBodyTransform;

@ClassDocumentation("This message commands the controller to move in taskspace the pelvis to the desired pose (position & orientation) while going through the specified waypoints."
      + " A third order polynomial function is used to interpolate positions and a hermite based curve (third order) is used to interpolate the orientations."
      + " To excute a single straight line trajectory to reach a desired pelvis pose, set only one waypoint with zero velocity and its time to be equal to the desired trajectory time."
      + " Note that the pelvis position is limited keep the robot's balance (center of mass has to remain inside the support polygon)."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.")
public class PelvisTrajectoryMessage extends IHMCRosApiPacket<PelvisTrajectoryMessage> implements TransformableDataObject<PelvisTrajectoryMessage>, VisualizablePacket
{
   @FieldDocumentation("List of waypoints (in taskpsace) to go through while executing the trajectory. All the information contained in these waypoints needs to be expressed in world frame.")
   public SE3WaypointMessage[] taskspaceWaypoints;

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public PelvisTrajectoryMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Clone constructor.
    * @param pelvisTrajectoryMessage message to clone.
    */
   public PelvisTrajectoryMessage(PelvisTrajectoryMessage pelvisTrajectoryMessage)
   {
      setUniqueId(pelvisTrajectoryMessage.getUniqueId());
      setDestination(pelvisTrajectoryMessage.getDestination());
      taskspaceWaypoints = new SE3WaypointMessage[pelvisTrajectoryMessage.getNumberOfWaypoints()];
      for (int i = 0; i < getNumberOfWaypoints(); i++)
         taskspaceWaypoints[i] = new SE3WaypointMessage(pelvisTrajectoryMessage.taskspaceWaypoints[i]);
   }

   /**
    * Use this constructor to execute a straight line trajectory in taskspace.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredPosition desired pelvis position expressed in world frame.
    * @param desiredOrientation desired pelvis orientation expressed in world frame.
    */
   public PelvisTrajectoryMessage(double trajectoryTime, Point3d desiredPosition, Quat4d desiredOrientation)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      Vector3d zeroLinearVelocity = new Vector3d();
      Vector3d zeroAngularVelocity = new Vector3d();
      taskspaceWaypoints = new SE3WaypointMessage[] {new SE3WaypointMessage(trajectoryTime, desiredPosition, desiredOrientation, zeroLinearVelocity, zeroAngularVelocity)};
   }

   /**
    * Use this constructor to build a message with more than one waypoint.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * This constructor only allocates memory for the waypoints, you need to call either {@link #setWaypoint(int, double, Point3d, Quat4d)} or {@link #setWaypoint(int, double, Point3d, Quat4d, Vector3d, Vector3d)} for each waypoint afterwards.
    * @param numberOfWaypoints number of waypoints that will be sent to the controller.
    */
   public PelvisTrajectoryMessage(int numberOfWaypoints)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      taskspaceWaypoints = new SE3WaypointMessage[numberOfWaypoints];
   }

   /**
    * Create a waypoint.
    * @param waypointIndex index of the waypoint to create.
    * @param time time at which the waypoint has to be reached. The time is relative to when the trajectory starts.
    * @param position define the desired 3D position to be reached at this waypoint. It is expressed in world frame.
    * @param orientation define the desired 3D orientation to be reached at this waypoint. It is expressed in world frame.
    * @param linearVelocity define the desired 3D linear velocity to be reached at this waypoint. It is expressed in world frame.
    * @param angularVelocity define the desired 3D angular velocity to be reached at this waypoint. It is expressed in world frame.
    */
   public void setWaypoint(int waypointIndex, double time, Point3d position, Quat4d orientation, Vector3d linearVelocity, Vector3d angularVelocity)
   {
      rangeCheck(waypointIndex);
      taskspaceWaypoints[waypointIndex] = new SE3WaypointMessage(time, position, orientation, linearVelocity, angularVelocity);
   }

   public int getNumberOfWaypoints()
   {
      return taskspaceWaypoints.length;
   }

   public SE3WaypointMessage getWaypoint(int waypointIndex)
   {
      rangeCheck(waypointIndex);
      return taskspaceWaypoints[waypointIndex];
   }

   public SE3WaypointMessage[] getWaypoints()
   {
      return taskspaceWaypoints;
   }

   private void rangeCheck(int waypointIndex)
   {
      if (waypointIndex >= getNumberOfWaypoints() || waypointIndex < 0)
         throw new IndexOutOfBoundsException("Waypoint index: " + waypointIndex + ", number of waypoints: " + getNumberOfWaypoints());
   }

   @Override
   public boolean epsilonEquals(PelvisTrajectoryMessage other, double epsilon)
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
   public PelvisTrajectoryMessage transform(RigidBodyTransform transform)
   {
      PelvisTrajectoryMessage transformedPelvisTrajectoryMessage = new PelvisTrajectoryMessage(getNumberOfWaypoints());

      for (int i = 0; i < getNumberOfWaypoints(); i++)
         transformedPelvisTrajectoryMessage.taskspaceWaypoints[i] = taskspaceWaypoints[i].transform(transform);

      return transformedPelvisTrajectoryMessage;
   }

   @Override
   public String toString()
   {
      if (taskspaceWaypoints != null)
         return "Pelvis SE3 trajectory: number of SE3 waypoints = " + getNumberOfWaypoints();
      else
         return "Pelvis SE3 trajectory: no SE3 waypoints";
   }
}
