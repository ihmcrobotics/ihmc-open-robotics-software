package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.QueueableMessage;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.JointspaceTrajectoryMessage;
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
public class ArmTrajectoryMessage extends Packet<ArmTrajectoryMessage>
{
   @RosExportedField(documentation = "Specifies the side of the robot that will execute the trajectory.")
   public RobotSide robotSide;
   @RosExportedField(documentation = "Trajectories for each joint.")
   public JointspaceTrajectoryMessage jointspaceTrajectory;

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public ArmTrajectoryMessage()
   {
      jointspaceTrajectory = new JointspaceTrajectoryMessage();
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Clone constructor.
    * @param armTrajectoryMessage message to clone.
    */
   public ArmTrajectoryMessage(ArmTrajectoryMessage armTrajectoryMessage)
   {
      jointspaceTrajectory = new JointspaceTrajectoryMessage(armTrajectoryMessage.jointspaceTrajectory);
      robotSide = armTrajectoryMessage.robotSide;
      setUniqueId(armTrajectoryMessage.getUniqueId());
   }

   public ArmTrajectoryMessage(RobotSide robotSide, JointspaceTrajectoryMessage jointspaceTrajectoryMessage)
   {
      jointspaceTrajectory = new JointspaceTrajectoryMessage(jointspaceTrajectoryMessage);
      this.robotSide = robotSide;
      setUniqueId(jointspaceTrajectoryMessage.getUniqueId());
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
      jointspaceTrajectory = new JointspaceTrajectoryMessage(trajectoryTime, desiredJointPositions);
      this.robotSide = robotSide;
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }
   
   /**
    * Use this constructor to go straight to the given end points using the specified qp weights.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param robotSide is used to define which arm is performing the trajectory.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredJointPositions desired joint positions. The array length should be equal to the number of arm joints.
    * @param weights the qp weights for the joint accelerations. If any index is set to NaN, that joint will use the controller default weight
    */
   public ArmTrajectoryMessage(RobotSide robotSide, double trajectoryTime, double[] desiredJointPositions, double[] weights)
   {
      jointspaceTrajectory = new JointspaceTrajectoryMessage(trajectoryTime, desiredJointPositions, weights);
      this.robotSide = robotSide;
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Create a message using the given joint trajectory points.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param robotSide is used to define which arm is performing the trajectory.
    * @param jointTrajectory1DListMessages joint trajectory points to be executed.
    */
   public ArmTrajectoryMessage(RobotSide robotSide, OneDoFJointTrajectoryMessage[] jointTrajectory1DListMessages)
   {
      jointspaceTrajectory = new JointspaceTrajectoryMessage(jointTrajectory1DListMessages);
      this.robotSide = robotSide;
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
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
      jointspaceTrajectory = new JointspaceTrajectoryMessage(numberOfJoints);
      this.robotSide = robotSide;
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
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
      jointspaceTrajectory = new JointspaceTrajectoryMessage(numberOfJoints, numberOfTrajectoryPoints);
      this.robotSide = robotSide;
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void setUniqueId(long uniqueId)
   {
      super.setUniqueId(uniqueId);
      if (jointspaceTrajectory != null)
         jointspaceTrajectory.setUniqueId(uniqueId);
   }

   public void set(ArmTrajectoryMessage other)
   {
      robotSide = other.robotSide;
      jointspaceTrajectory.set(other.jointspaceTrajectory);
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public void setJointspaceTrajectory(JointspaceTrajectoryMessage jointspaceTrajectory)
   {
      this.jointspaceTrajectory = jointspaceTrajectory;
   }

   public JointspaceTrajectoryMessage getJointspaceTrajectory()
   {
      return jointspaceTrajectory;
   }

   public QueueableMessage getQueueingProperties()
   {
      return jointspaceTrajectory.getQueueingProperties();
   }

   @Override
   public boolean epsilonEquals(ArmTrajectoryMessage other, double epsilon)
   {
      if (!this.robotSide.equals(other.robotSide))
         return false;
      if (!jointspaceTrajectory.epsilonEquals(other.jointspaceTrajectory, epsilon))
         return false;
      return true;
   }

   @Override
   public String toString()
   {
      if (jointspaceTrajectory.jointTrajectoryMessages != null)
         return "Arm 1D trajectories: number of joints = " + jointspaceTrajectory.getNumberOfJoints() + ", robotSide = " + robotSide;
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
