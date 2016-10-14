package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.TrackablePacket;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.humanoidRobotics.communication.packets.TrajectoryPoint1DMessage;
import us.ihmc.robotics.robotSide.RobotSide;

@RosMessagePacket(documentation =
      "This message commands the controller to move an arm in jointspace to the desired joint angles while going through the specified trajectory points."
      + " A third order polynomial function is used to interpolate between trajectory points."
      + " The jointTrajectoryMessages can have different waypoint times and different number of waypoints."
      + " If a joint trajectory message is empty, the controller will hold the last desired joint position while executing the other joint trajectories."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.",
      rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
      topic = "/control/arm_trajectory")
public class ArmTrajectoryMessage extends TrackablePacket<ArmTrajectoryMessage> implements VisualizablePacket
{
   @RosExportedField(documentation = "Specifies the side of the robot that will execute the trajectory.")
   public RobotSide robotSide;
   @RosExportedField(documentation = "List of trajectory points per joint."
         + " The expected joint ordering is from the closest joint to the chest to the closest joint to the hand.")
   public OneDoFJointTrajectoryMessage[] jointTrajectoryMessages;
   @RosExportedField(documentation = "When OVERRIDE is chosen:"
         + "\n - The time of the first trajectory point can be zero, in which case the controller will start directly at the first trajectory point."
         + " Otherwise the controller will prepend a first trajectory point at the current desired position."
         + "\n When QUEUE is chosen:"
         + "\n - The message must carry the ID of the message it should be queued to."
         + "\n - The very first message of a list of queued messages has to be an OVERRIDE message."
         + "\n - The trajectory point times are relative to the the last trajectory point time of the previous message."
         + "\n - The controller will queue the joint trajectory messages as a per joint basis."
         + " The first trajectory point has to be greater than zero.")
   public ExecutionMode executionMode;
   @RosExportedField(documentation = "Only needed when using QUEUE mode, it refers to the message Id to which this message should be queued to."
         + " It is used by the controller to ensure that no message has been lost on the way."
         + " If a message appears to be missing (previousMessageId different from the last message ID received by the controller), the motion is aborted."
         + " If previousMessageId == 0, the controller will not check for the ID of the last received message.")
   public long previousMessageId = INVALID_MESSAGE_ID;

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public ArmTrajectoryMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      executionMode = ExecutionMode.OVERRIDE;
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
      jointTrajectoryMessages = new OneDoFJointTrajectoryMessage[armTrajectoryMessage.getNumberOfJoints()];

      for (int i = 0; i < getNumberOfJoints(); i++)
         jointTrajectoryMessages[i] = new OneDoFJointTrajectoryMessage(armTrajectoryMessage.jointTrajectoryMessages[i]);

      executionMode = armTrajectoryMessage.executionMode;
      previousMessageId = armTrajectoryMessage.previousMessageId;
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
      jointTrajectoryMessages = new OneDoFJointTrajectoryMessage[desiredJointPositions.length];
      for (int jointIndex = 0; jointIndex < getNumberOfJoints(); jointIndex++)
         jointTrajectoryMessages[jointIndex] = new OneDoFJointTrajectoryMessage(trajectoryTime, desiredJointPositions[jointIndex]);

      executionMode = ExecutionMode.OVERRIDE;
   }

   /**
    * Create a message using the given joint trajectory points.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param robotSide is used to define which arm is performing the trajectory.
    * @param jointTrajectory1DListMessages joint trajectory points to be executed.
    */
   public ArmTrajectoryMessage(RobotSide robotSide, OneDoFJointTrajectoryMessage[] jointTrajectory1DListMessages)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.robotSide = robotSide;
      this.jointTrajectoryMessages = jointTrajectory1DListMessages;
      executionMode = ExecutionMode.OVERRIDE;
   }

   /**
    * Use this constructor to build a message with more than one trajectory point.
    * This constructor only allocates memory for the trajectories, you need to call {@link #setTrajectory1DMessage(int, OneDoFJointTrajectoryMessage)} for each joint afterwards.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param robotSide is used to define which arm is performing the trajectory.
    * @param numberOfJoints number of joints that will be executing the message.
    */
   public ArmTrajectoryMessage(RobotSide robotSide, int numberOfJoints)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.robotSide = robotSide;
      jointTrajectoryMessages = new OneDoFJointTrajectoryMessage[numberOfJoints];
      executionMode = ExecutionMode.OVERRIDE;
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
      jointTrajectoryMessages = new OneDoFJointTrajectoryMessage[numberOfJoints];
      for (int i = 0; i < numberOfJoints; i++)
         jointTrajectoryMessages[i] = new OneDoFJointTrajectoryMessage(numberOfTrajectoryPoints);

      executionMode = ExecutionMode.OVERRIDE;
   }

   /**
    * Set the trajectory points to be executed by this joint.
    * @param jointIndex index of the joint that will go through the trajectory points.
    * @param trajectory1DMessage joint trajectory points to be executed.
    */
   public void setTrajectory1DMessage(int jointIndex, OneDoFJointTrajectoryMessage trajectory1DMessage)
   {
      rangeCheck(jointIndex);
      jointTrajectoryMessages[jointIndex] = trajectory1DMessage;
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
      jointTrajectoryMessages[jointIndex].setTrajectoryPoint(trajectoryPointIndex, time, position, velocity);
   }

   /**
    * Set how the controller should consume this message:
    * <li> {@link ExecutionMode#OVERRIDE}: this message will override any previous message, including canceling any active execution of a message.
    * <li> {@link ExecutionMode#QUEUE}: this message is queued and will be executed once all the previous messages are done.
    * @param executionMode
    * @param previousMessageId when queuing, one needs to provide the ID of the message this message should be queued to.
    */
   public void setExecutionMode(ExecutionMode executionMode, long previousMessageId)
   {
      this.executionMode = executionMode;
      this.previousMessageId = previousMessageId;
   }

   public int getNumberOfJoints()
   {
      return jointTrajectoryMessages.length;
   }

   public int getNumberOfJointTrajectoryPoints(int jointIndex)
   {
      rangeCheck(jointIndex);
      return jointTrajectoryMessages[jointIndex].getNumberOfTrajectoryPoints();
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public OneDoFJointTrajectoryMessage getJointTrajectoryPointList(int jointIndex)
   {
      rangeCheck(jointIndex);
      return jointTrajectoryMessages[jointIndex];
   }

   public OneDoFJointTrajectoryMessage[] getTrajectoryPointLists()
   {
      return jointTrajectoryMessages;
   }

   public TrajectoryPoint1DMessage getJointTrajectoryPoint(int jointIndex, int trajectoryPointIndex)
   {
      rangeCheck(jointIndex);
      return jointTrajectoryMessages[jointIndex].getTrajectoryPoint(trajectoryPointIndex);
   }

   public void getFinalJointAngles(double[] finalJointAnglesToPack)
   {
      for (int i = 0; i < getNumberOfJoints(); i++)
      {
         finalJointAnglesToPack[i] = jointTrajectoryMessages[i].getLastTrajectoryPoint().position;
      }
   }

   public double getTrajectoryTime()
   {
      double trajectoryTime = 0.0;
      for (int i = 0; i < getNumberOfJoints(); i++)
         trajectoryTime = Math.max(trajectoryTime, jointTrajectoryMessages[i].getLastTrajectoryPoint().time);
      return trajectoryTime;
   }

   public ExecutionMode getExecutionMode()
   {
      return executionMode;
   }

   public long getPreviousMessageId()
   {
      return previousMessageId;
   }

   private void rangeCheck(int jointIndex)
   {
      if (jointIndex >= getNumberOfJoints() || jointIndex < 0)
         throw new IndexOutOfBoundsException("Joint index: " + jointIndex + ", number of joints: " + getNumberOfJoints());
   }

   @Override
   public boolean epsilonEquals(ArmTrajectoryMessage other, double epsilon)
   {
      if (!this.robotSide.equals(other.robotSide) || this.jointTrajectoryMessages.length != other.jointTrajectoryMessages.length)
      {
         return false;
      }

      if (executionMode != other.executionMode)
         return false;

      if (executionMode == ExecutionMode.OVERRIDE && previousMessageId != other.previousMessageId)
         return false;

      for (int i = 0; i < this.jointTrajectoryMessages.length; i++)
      {
         if (!this.jointTrajectoryMessages[i].epsilonEquals(other.jointTrajectoryMessages[i], epsilon))
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
         setTrajectory1DMessage(i, new OneDoFJointTrajectoryMessage(random));

      executionMode = ExecutionMode.values[random.nextInt(2)];
   }

   @Override
   public String toString()
   {
      if (jointTrajectoryMessages != null)
         return "Arm 1D trajectories: number of joints = " + getNumberOfJoints() + ", robotSide = " + robotSide;
      else
         return "Arm 1D trajectories: no joint trajectory, robotSide = " + robotSide;
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateArmTrajectoryMessage(this);
   }
}
