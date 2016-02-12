package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.humanoidRobotics.communication.packets.Waypoint1DMessage;
import us.ihmc.robotics.robotSide.RobotSide;

@ClassDocumentation("This message commands the controller to move an arm in jointspace to the desired joint angles while going through the specified waypoints."
      + " A third order polynomial function is used to interpolate between waypoints."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.")
public class ArmTrajectoryMessage extends IHMCRosApiPacket<ArmTrajectoryMessage> implements VisualizablePacket
{
   @FieldDocumentation("Specifies the side of the robot that will execute the trajectory.")
   public RobotSide robotSide;
   @FieldDocumentation("List of points in the trajectory."
         + " The expected joint ordering is from the closest joint to the chest to the closest joint to the hand.")
   public Trajectory1DMessage[] jointTrajectory1DMessages;

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public ArmTrajectoryMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Clone constructor.
    * @param armTrajectoryMessage message to clone.
    */
   public ArmTrajectoryMessage(ArmTrajectoryMessage armTrajectoryMessage)
   {
      setUniqueId(armTrajectoryMessage.getUniqueId());
      setDestination(armTrajectoryMessage.getDestination());
      robotSide = armTrajectoryMessage.robotSide;
      jointTrajectory1DMessages = new Trajectory1DMessage[armTrajectoryMessage.getNumberOfJoints()];

      for (int i = 0; i < getNumberOfJoints(); i++)
         jointTrajectory1DMessages[i] = new Trajectory1DMessage(armTrajectoryMessage.jointTrajectory1DMessages[i]);
   }

   /**
    * Use this constructor to go straight to the given end points.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param robotSide is used to define which arm is performing the trajectory.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredJointPositions desired joint positions. The array length should be equal to the number of arm joints.
    */
   public ArmTrajectoryMessage(RobotSide robotSide, double trajectoryTime, double[] desiredJointPositions)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.robotSide = robotSide;
      jointTrajectory1DMessages = new Trajectory1DMessage[desiredJointPositions.length];
      for (int jointIndex = 0; jointIndex < getNumberOfJoints(); jointIndex++)
         jointTrajectory1DMessages[jointIndex] = new Trajectory1DMessage(trajectoryTime, desiredJointPositions[jointIndex]);
   }

   /**
    * Create a message using the given joint trajectory waypoints.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param robotSide is used to define which arm is performing the trajectory.
    * @param jointTrajectory1DMessages joint trajectory waypoints to be executed.
    */
   public ArmTrajectoryMessage(RobotSide robotSide, Trajectory1DMessage[] jointTrajectory1DMessages)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.robotSide = robotSide;
      this.jointTrajectory1DMessages = jointTrajectory1DMessages;
   }

   /**
    * Use this constructor to build a message with more than one waypoint.
    * This constructor only allocates memory for the trajectories, you need to call {@link #setTrajectory1DMessage(int, Trajectory1DMessage)} for each joint afterwards.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param robotSide is used to define which arm is performing the trajectory.
    * @param numberOfJoints number of joints that will be executing the message.
    */
   public ArmTrajectoryMessage(RobotSide robotSide, int numberOfJoints)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.robotSide = robotSide;
      jointTrajectory1DMessages = new Trajectory1DMessage[numberOfJoints];
   }

   /**
    * Use this constructor to build a message with more than one waypoint.
    * This constructor only allocates memory for the trajectories and their waypoints, you need to call {@link #setWaypoint(int, int, double, double, double)} for each joint and waypoint afterwards.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param robotSide is used to define which arm is performing the trajectory.
    * @param numberOfJoints number of joints that will be executing the message.
    * @param numberOfWaypoints number of waypoints that will be sent to the controller.
    */
   public ArmTrajectoryMessage(RobotSide robotSide, int numberOfJoints, int numberOfWaypoints)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.robotSide = robotSide;
      jointTrajectory1DMessages = new Trajectory1DMessage[numberOfJoints];
      for (int i = 0; i < numberOfWaypoints; i++)
         jointTrajectory1DMessages[i] = new Trajectory1DMessage(numberOfWaypoints);
   }

   /**
    * Set the trajectory waypoints to be executed by this joint.
    * @param jointIndex index of the joint that will go through the waypoint.
    * @param trajectory1DMessage joint trajectory waypoints to be executed.
    */
   public void setTrajectory1DMessage(int jointIndex, Trajectory1DMessage trajectory1DMessage)
   {
      rangeCheck(jointIndex);
      jointTrajectory1DMessages[jointIndex] = trajectory1DMessage;
   }

   /**
    * Create a waypoint.
    * @param jointIndex index of the joint that will go through the waypoint.
    * @param waypointIndex index of the waypoint to create.
    * @param time time at which the waypoint has to be reached. The time is relative to when the trajectory starts.
    * @param position define the desired 1D position to be reached at this waypoint.
    * @param velocity define the desired 1D velocity to be reached at this waypoint.
    */
   public void setWaypoint(int jointIndex, int waypointIndex, double time, double position, double velocity)
   {
      rangeCheck(jointIndex);
      jointTrajectory1DMessages[jointIndex].setWaypoint(waypointIndex, time, position, velocity);
   }

   public int getNumberOfJoints()
   {
      return jointTrajectory1DMessages.length;
   }

   public int getNumberOfWaypointsForJointTrajectory(int jointIndex)
   {
      rangeCheck(jointIndex);
      return jointTrajectory1DMessages[jointIndex].getNumberOfWaypoints();
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public Trajectory1DMessage getJointTrajectory(int jointIndex)
   {
      rangeCheck(jointIndex);
      return jointTrajectory1DMessages[jointIndex];
   }

   public Trajectory1DMessage[] getJointTrajectory()
   {
      return jointTrajectory1DMessages;
   }

   public Waypoint1DMessage getJointTrajectoryWaypoint(int jointIndex, int waypointIndex)
   {
      rangeCheck(jointIndex);
      return jointTrajectory1DMessages[jointIndex].getWaypoint(waypointIndex);
   }

   private void rangeCheck(int jointIndex)
   {
      if (jointIndex >= getNumberOfJoints() || jointIndex < 0)
         throw new IndexOutOfBoundsException("Joint index: " + jointIndex + ", number of joints: " + getNumberOfJoints());
   }

   @Override
   public boolean epsilonEquals(ArmTrajectoryMessage other, double epsilon)
   {
      if (!this.robotSide.equals(other.robotSide) || this.jointTrajectory1DMessages.length != other.jointTrajectory1DMessages.length)
      {
         return false;
      }

      for (int i = 0; i < this.jointTrajectory1DMessages.length; i++)
      {
         if (!this.jointTrajectory1DMessages[i].epsilonEquals(other.jointTrajectory1DMessages[i], epsilon))
         {
            return false;
         }
      }

      return true;
   }

   public ArmTrajectoryMessage(Random random)
   {
      this(random.nextBoolean() ? RobotSide.LEFT : RobotSide.RIGHT, random.nextInt(10) + 1);

      for (int i = 0; i < getNumberOfJoints(); i++)
         setTrajectory1DMessage(i, new Trajectory1DMessage(random));
   }

   @Override
   public String toString()
   {
      if (jointTrajectory1DMessages != null)
         return "Arm 1D trajectories: number of joints = " + getNumberOfJoints() + ", robotSide = " + robotSide;
      else
         return "Arm 1D trajectories: no joint trajectory, robotSide = " + robotSide;
   }
}
