package us.ihmc.humanoidRobotics.communication.packets.walking;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.humanoidRobotics.communication.TransformableDataObject;
import us.ihmc.humanoidRobotics.communication.packets.SE3WaypointMessage;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;

@ClassDocumentation("This message commands the controller first to unload if necessary and then to move in taskspace a foot to the desired pose (position & orientation) while going through the specified waypoints."
      + " A third order polynomial function is used to interpolate positions and a hermite based curve (third order) is used to interpolate the orientations."
      + " To excute a single straight line trajectory to reach a desired foot pose, set only one waypoint with zero velocity and its time to be equal to the desired trajectory time.")
public class FootTrajectoryMessage extends IHMCRosApiPacket<FootTrajectoryMessage> implements TransformableDataObject<FootTrajectoryMessage>, VisualizablePacket
{
   @FieldDocumentation("Specifies the which foot will execute the trajectory.")
   public RobotSide robotSide;
   @FieldDocumentation("List of waypoints (in taskpsace) to go through while executing the trajectory. All the information contained in these waypoints needs to be expressed in world frame.")
   public SE3WaypointMessage[] taskspaceWaypoints;

   /**
    * Empty constructor for serialization.
    */
   public FootTrajectoryMessage()
   {
   }

   public FootTrajectoryMessage(FootTrajectoryMessage footTrajectoryMessage)
   {
      robotSide = footTrajectoryMessage.robotSide;
      taskspaceWaypoints = new SE3WaypointMessage[footTrajectoryMessage.getNumberOfWaypoints()];
      for (int i = 0; i < getNumberOfWaypoints(); i++)
         taskspaceWaypoints[i] = new SE3WaypointMessage(footTrajectoryMessage.taskspaceWaypoints[i]);
   }

   /**
    * Use this constructor to execute a straight line trajectory in taskspace. The chest is used as the base for the control.
    * @param robotSide is used to define which foot is performing the trajectory.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredPosition desired foot position expressed in world frame.
    * @param desiredOrientation desired foot orientation expressed in world frame.
    */
   public FootTrajectoryMessage(RobotSide robotSide, double trajectoryTime, Point3d desiredPosition, Quat4d desiredOrientation)
   {
      this.robotSide = robotSide;
      Vector3d zeroLinearVelocity = new Vector3d();
      Vector3d zeroAngularVelocity = new Vector3d();
      taskspaceWaypoints = new SE3WaypointMessage[] {new SE3WaypointMessage(trajectoryTime, desiredPosition, desiredOrientation, zeroLinearVelocity, zeroAngularVelocity)};
   }

   /**
    * Use this constructor to build a message with more than one waypoint.
    * This constructor only allocates memory for the waypoints, you need to call either {@link #setWaypoint(int, double, Point3d, Quat4d)} or {@link #setWaypoint(int, double, Point3d, Quat4d, Vector3d, Vector3d)} for each waypoint afterwards.
    * @param robotSide is used to define which foot is performing the trajectory.
    * @param numberOfWaypoints number of waypoints that will be sent to the controller.
    */
   public FootTrajectoryMessage(RobotSide robotSide, int numberOfWaypoints)
   {
      this.robotSide = robotSide;
      taskspaceWaypoints = new SE3WaypointMessage[numberOfWaypoints];
   }

   public void setWaypoint(int waypointIndex, double time, Point3d position, Quat4d orientation, Vector3d linearVelocity, Vector3d angularVelocity)
   {
      rangeCheck(waypointIndex);
      taskspaceWaypoints[waypointIndex] = new SE3WaypointMessage(time, position, orientation, linearVelocity, angularVelocity);
   }

   public int getNumberOfWaypoints()
   {
      return taskspaceWaypoints.length;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
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
   public boolean epsilonEquals(FootTrajectoryMessage other, double epsilon)
   {
      if (robotSide != other.robotSide)
         return false;
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
   public FootTrajectoryMessage transform(RigidBodyTransform transform)
   {
      FootTrajectoryMessage transformedFootTrajectoryMessage = new FootTrajectoryMessage(robotSide, getNumberOfWaypoints());

      for (int i = 0; i < getNumberOfWaypoints(); i++)
         transformedFootTrajectoryMessage.taskspaceWaypoints[i] = taskspaceWaypoints[i].transform(transform);

      return transformedFootTrajectoryMessage;
   }
}
