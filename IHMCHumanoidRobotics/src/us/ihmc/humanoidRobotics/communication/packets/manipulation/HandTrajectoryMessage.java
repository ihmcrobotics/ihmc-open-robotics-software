package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.humanoidRobotics.communication.packets.SE3Waypoint;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.DocumentedEnum;

@ClassDocumentation("This message commands the controller to move in taskspace a hand to the desired pose (position & orientation) while going through the specified waypoints."
      + " A fifth order polynomial function is used to interpolate from one waypoint to the next one."
      + " If the velocities are close to zero at the waypoints, the trajectory will look like a straight line path between each pair of waypoints.")
public class HandTrajectoryMessage extends IHMCRosApiPacket<HandTrajectoryMessage> implements VisualizablePacket
{
   public enum BaseForControl implements DocumentedEnum<BaseForControl>
   {
      CHEST, WORLD;

      @Override
      public String getDocumentation(BaseForControl var)
      {
         switch (var)
         {
         case CHEST:
            return "The hand is controlled with respect to the chest. In other words, the controlled hand moves along with the chest.";
         case WORLD:
            return "The hand is controlled with respect to the estimated world. In other words, the controlled hand will remain fixed in world even if the robot starts moving.";

         default:
            return "No documentation available.";
         }
      }

      @Override
      public BaseForControl[] getDocumentedValues()
      {
         return values();
      }
   }

   @FieldDocumentation("Specifies the which hand will execute the trajectory.")
   public RobotSide robotSide;
   @FieldDocumentation("Specify whether the pose should be held with respect to the world or the chest. "
         + "Note that in any case the desired hand pose must be expressed in world frame.")
   public BaseForControl base;
   @FieldDocumentation("List of waypoints (in taskpsace) to go through while executing the trajectory.")
   public SE3Waypoint[] taskspaceWaypoints;

   public HandTrajectoryMessage()
   {
   }

   public HandTrajectoryMessage(HandTrajectoryMessage handTrajectoryMessage)
   {
      robotSide = handTrajectoryMessage.robotSide;
      taskspaceWaypoints = new SE3Waypoint[handTrajectoryMessage.getNumberOfWaypoints()];
      for (int i = 0; i < getNumberOfWaypoints(); i++)
         taskspaceWaypoints[i] = new SE3Waypoint(handTrajectoryMessage.taskspaceWaypoints[i]);
   }

   /**
    * Use this constructor to execute a straight line trajectory in taskspace. The chest is used as the base for the control.
    * @param robotSide is used to define which hand is performing the trajectory.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredPosition desired hand position expressed in world frame.
    * @param desiredOrientation desired hand orientation expressed in world frame.
    */
   public HandTrajectoryMessage(RobotSide robotSide, double trajectoryTime, Point3d desiredPosition, Quat4d desiredOrientation)
   {
      this(robotSide, BaseForControl.CHEST, trajectoryTime, desiredPosition, desiredOrientation);
   }

   /**
    * Use this constructor to execute a straight line trajectory in taskspace.
    * @param robotSide is used to define which hand is performing the trajectory.
    * @param base define with respect to what base the hand is controlled.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredPosition desired hand position expressed in world frame.
    * @param desiredOrientation desired hand orientation expressed in world frame.
    */
   public HandTrajectoryMessage(RobotSide robotSide, BaseForControl base, double trajectoryTime, Point3d desiredPosition, Quat4d desiredOrientation)
   {
      this.robotSide = robotSide;
      this.base = base;
      Vector3d zeroLinearVelocity = new Vector3d();
      Vector3d zeroAngularVelocity = new Vector3d();
      taskspaceWaypoints = new SE3Waypoint[] {new SE3Waypoint(trajectoryTime, desiredPosition, desiredOrientation, zeroLinearVelocity, zeroAngularVelocity)};
   }

   /**
    * Use this constructor to build a message with more than one waypoint.
    * This constructor only allocates memory for the waypoints, you need to call either {@link #setWaypoint(int, double, Point3d, Quat4d)} or {@link #setWaypoint(int, double, Point3d, Quat4d, Vector3d, Vector3d)} for each waypoint afterwards.
    * @param robotSide is used to define which hand is performing the trajectory.
    * @param base define with respect to what base the hand is controlled.
    * @param numberOfWaypoints number of waypoints that will be sent to the controller.
    */
   public HandTrajectoryMessage(RobotSide robotSide, BaseForControl base, int numberOfWaypoints)
   {
      this.robotSide = robotSide;
      this.base = base;
      taskspaceWaypoints = new SE3Waypoint[numberOfWaypoints];
   }

   public void setWaypoint(int waypointIndex, double time, Point3d position, Quat4d orientation)
   {
      rangeCheck(waypointIndex);
      taskspaceWaypoints[waypointIndex] = new SE3Waypoint(time, position, orientation, null, null);
   }

   public void setWaypoint(int waypointIndex, double time, Point3d position, Quat4d orientation, Vector3d linearVelocity, Vector3d angularVelocity)
   {
      rangeCheck(waypointIndex);
      taskspaceWaypoints[waypointIndex] = new SE3Waypoint(time, position, orientation, linearVelocity, angularVelocity);
   }

   public int getNumberOfWaypoints()
   {
      return taskspaceWaypoints.length;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public BaseForControl getBase()
   {
      return base;
   }

   public SE3Waypoint getTaskspaceWaypoint(int waypointIndex)
   {
      rangeCheck(waypointIndex);
      return taskspaceWaypoints[waypointIndex];
   }

   public SE3Waypoint[] getTaskspaceWaypoints()
   {
      return taskspaceWaypoints;
   }

   private void rangeCheck(int waypointIndex)
   {
      if (waypointIndex >= getNumberOfWaypoints() || waypointIndex < 0)
         throw new IndexOutOfBoundsException("Waypoint index: " + waypointIndex + ", number of waypoints: " + getNumberOfWaypoints());
   }

   @Override
   public boolean epsilonEquals(HandTrajectoryMessage other, double epsilon)
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
}
