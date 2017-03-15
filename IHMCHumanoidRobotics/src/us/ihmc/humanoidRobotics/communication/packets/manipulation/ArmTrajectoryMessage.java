package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.AbstractJointspaceTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.robotics.robotSide.RobotSide;

@RosMessagePacket(documentation =
      "This message commands the controller to move an arm in jointspace to the desired joint angles while going through the specified trajectory points."
      + " A third order polynomial function is used to interpolate between trajectory points."
      + " The jointTrajectoryMessages can have different waypoint times and different number of waypoints."
      + " If a joint trajectory message is empty, the controller will hold the last desired joint position while executing the other joint trajectories."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.",
      rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
      topic = "/control/arm_trajectory")
public class ArmTrajectoryMessage extends AbstractJointspaceTrajectoryMessage<ArmTrajectoryMessage> implements VisualizablePacket
{
   @RosExportedField(documentation = "Specifies the side of the robot that will execute the trajectory.")
   public RobotSide robotSide;

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public ArmTrajectoryMessage()
   {
      super();
   }

   /**
    * Clone constructor.
    * @param armTrajectoryMessage message to clone.
    */
   public ArmTrajectoryMessage(ArmTrajectoryMessage armTrajectoryMessage)
   {
      super(armTrajectoryMessage);
      robotSide = armTrajectoryMessage.robotSide;
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
      super(trajectoryTime, desiredJointPositions);
      this.robotSide = robotSide;
   }

   /**
    * Create a message using the given joint trajectory points.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param robotSide is used to define which arm is performing the trajectory.
    * @param jointTrajectory1DListMessages joint trajectory points to be executed.
    */
   public ArmTrajectoryMessage(RobotSide robotSide, OneDoFJointTrajectoryMessage[] jointTrajectory1DListMessages)
   {
      super(jointTrajectory1DListMessages);
      this.robotSide = robotSide;
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
      super(numberOfJoints);
      this.robotSide = robotSide;
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
      super(numberOfJoints, numberOfTrajectoryPoints);
      this.robotSide = robotSide;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   @Override
   public boolean epsilonEquals(ArmTrajectoryMessage other, double epsilon)
   {
      if (!this.robotSide.equals(other.robotSide))
         return false;

      return super.epsilonEquals(other, epsilon);
   }

   public ArmTrajectoryMessage(Random random)
   {
      super(random);
      this.robotSide = RandomNumbers.nextEnum(random, RobotSide.class);
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
