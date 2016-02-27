package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.IHMCRosApiMessage;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.humanoidRobotics.communication.packets.TrajectoryPoint1DMessage;
import us.ihmc.robotics.robotSide.RobotSide;

@ClassDocumentation("This message commands the controller to move an arm in jointspace to the desired joint angles while going through the specified trajectory points."
      + " A third order polynomial function is used to interpolate between trajectory points."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.")
public class ArmTrajectoryMessage extends IHMCRosApiMessage<ArmTrajectoryMessage> implements VisualizablePacket
{
   @FieldDocumentation("Specifies the side of the robot that will execute the trajectory.")
   public RobotSide robotSide;
   @FieldDocumentation("List of points in the trajectory."
         + " The expected joint ordering is from the closest joint to the chest to the closest joint to the hand.")
   public ArmOneJointTrajectoryMessage[] jointTrajectory1DListMessages;

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
      jointTrajectory1DListMessages = new ArmOneJointTrajectoryMessage[armTrajectoryMessage.getNumberOfJoints()];

      for (int i = 0; i < getNumberOfJoints(); i++)
         jointTrajectory1DListMessages[i] = new ArmOneJointTrajectoryMessage(armTrajectoryMessage.jointTrajectory1DListMessages[i]);
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
      jointTrajectory1DListMessages = new ArmOneJointTrajectoryMessage[desiredJointPositions.length];
      for (int jointIndex = 0; jointIndex < getNumberOfJoints(); jointIndex++)
         jointTrajectory1DListMessages[jointIndex] = new ArmOneJointTrajectoryMessage(trajectoryTime, desiredJointPositions[jointIndex]);
   }

   /**
    * Create a message using the given joint trajectory points.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param robotSide is used to define which arm is performing the trajectory.
    * @param jointTrajectory1DListMessages joint trajectory points to be executed.
    */
   public ArmTrajectoryMessage(RobotSide robotSide, ArmOneJointTrajectoryMessage[] jointTrajectory1DListMessages)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.robotSide = robotSide;
      this.jointTrajectory1DListMessages = jointTrajectory1DListMessages;
   }

   /**
    * Use this constructor to build a message with more than one trajectory point.
    * This constructor only allocates memory for the trajectories, you need to call {@link #setTrajectory1DMessage(int, ArmOneJointTrajectoryMessage)} for each joint afterwards.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param robotSide is used to define which arm is performing the trajectory.
    * @param numberOfJoints number of joints that will be executing the message.
    */
   public ArmTrajectoryMessage(RobotSide robotSide, int numberOfJoints)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.robotSide = robotSide;
      jointTrajectory1DListMessages = new ArmOneJointTrajectoryMessage[numberOfJoints];
   }

   /**
    * Use this constructor to build a message with more than one trajectory points.
    * This constructor only allocates memory for the trajectories, you need to call {@link #setTrajectoryPoint(int, int, double, double, double)} for each joint and trajectory point afterwards.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param robotSide is used to define which arm is performing the trajectory.
    * @param numberOfJoints number of joints that will be executing the message.
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the controller.
    */
   public ArmTrajectoryMessage(RobotSide robotSide, int numberOfJoints, int numberOfTrajectoryPoints)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.robotSide = robotSide;
      jointTrajectory1DListMessages = new ArmOneJointTrajectoryMessage[numberOfJoints];
      for (int i = 0; i < numberOfTrajectoryPoints; i++)
         jointTrajectory1DListMessages[i] = new ArmOneJointTrajectoryMessage(numberOfTrajectoryPoints);
   }

   /**
    * Set the trajectory points to be executed by this joint.
    * @param jointIndex index of the joint that will go through the trajectory points.
    * @param trajectory1DMessage joint trajectory points to be executed.
    */
   public void setTrajectory1DMessage(int jointIndex, ArmOneJointTrajectoryMessage trajectory1DMessage)
   {
      rangeCheck(jointIndex);
      jointTrajectory1DListMessages[jointIndex] = trajectory1DMessage;
   }

   /**
    * Create a trajectory point.
    * @param jointIndex index of the joint that will go through the trajectory point.
    * @param trajectoryPointIndex index of the trajectory point to create.
    * @param time time at which the trajectory point has to be reached. The time is relative to when the trajectory starts.
    * @param position define the desired 1D position to be reached at this trajectory point.
    * @param velocity define the desired 1D velocity to be reached at this trajectory point.
    */
   public void setTrajectoryPoint(int jointIndex, int trajectoryPointIndex, double time, double position, double velocity)
   {
      rangeCheck(jointIndex);
      jointTrajectory1DListMessages[jointIndex].setTrajectoryPoint(trajectoryPointIndex, time, position, velocity);
   }

   public int getNumberOfJoints()
   {
      return jointTrajectory1DListMessages.length;
   }

   public int getNumberOfJointTrajectoryPoints(int jointIndex)
   {
      rangeCheck(jointIndex);
      return jointTrajectory1DListMessages[jointIndex].getNumberOfTrajectoryPoints();
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public ArmOneJointTrajectoryMessage getJointTrajectoryPointList(int jointIndex)
   {
      rangeCheck(jointIndex);
      return jointTrajectory1DListMessages[jointIndex];
   }

   public ArmOneJointTrajectoryMessage[] getTrajectoryPointLists()
   {
      return jointTrajectory1DListMessages;
   }

   public TrajectoryPoint1DMessage getJointTrajectoryPoint(int jointIndex, int trajectoryPointIndex)
   {
      rangeCheck(jointIndex);
      return jointTrajectory1DListMessages[jointIndex].getTrajectoryPoint(trajectoryPointIndex);
   }

   private void rangeCheck(int jointIndex)
   {
      if (jointIndex >= getNumberOfJoints() || jointIndex < 0)
         throw new IndexOutOfBoundsException("Joint index: " + jointIndex + ", number of joints: " + getNumberOfJoints());
   }

   @Override
   public boolean epsilonEquals(ArmTrajectoryMessage other, double epsilon)
   {
      if (!this.robotSide.equals(other.robotSide) || this.jointTrajectory1DListMessages.length != other.jointTrajectory1DListMessages.length)
      {
         return false;
      }

      for (int i = 0; i < this.jointTrajectory1DListMessages.length; i++)
      {
         if (!this.jointTrajectory1DListMessages[i].epsilonEquals(other.jointTrajectory1DListMessages[i], epsilon))
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
         setTrajectory1DMessage(i, new ArmOneJointTrajectoryMessage(random));
   }

   @Override
   public String toString()
   {
      if (jointTrajectory1DListMessages != null)
         return "Arm 1D trajectories: number of joints = " + getNumberOfJoints() + ", robotSide = " + robotSide;
      else
         return "Arm 1D trajectories: no joint trajectory, robotSide = " + robotSide;
   }
}
