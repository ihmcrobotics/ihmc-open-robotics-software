package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.QueueableMessage;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.JointspaceTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.OneDoFJointTrajectoryMessage;

@RosMessagePacket(documentation =
      "This message commands the controller to move the neck in jointspace to the desired joint angles while going through the specified trajectory points."
      + " A third order polynomial function is used to interpolate between trajectory points."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.",
                  rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
                  topic = "/control/neck_trajectory")
public class NeckTrajectoryMessage extends Packet<NeckTrajectoryMessage>
{
   @RosExportedField(documentation = "Trajectories for each joint.")
   public JointspaceTrajectoryMessage jointspaceTrajectory;

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public NeckTrajectoryMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Clone constructor.
    * @param neckTrajectoryMessage message to clone.
    */
   public NeckTrajectoryMessage(NeckTrajectoryMessage neckTrajectoryMessage)
   {
      jointspaceTrajectory = new JointspaceTrajectoryMessage(neckTrajectoryMessage.jointspaceTrajectory);
      setUniqueId(neckTrajectoryMessage.getUniqueId());
   }

   public NeckTrajectoryMessage(JointspaceTrajectoryMessage jointspaceTrajectoryMessage)
   {
      jointspaceTrajectory = new JointspaceTrajectoryMessage(jointspaceTrajectoryMessage);
      setUniqueId(jointspaceTrajectoryMessage.getUniqueId());
   }

   /**
    * Use this constructor to go straight to the given end points.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredJointPositions desired joint positions. The array length should be equal to the number of joints.
    */
   public NeckTrajectoryMessage(double trajectoryTime, double[] desiredJointPositions)
   {
      jointspaceTrajectory = new JointspaceTrajectoryMessage(trajectoryTime, desiredJointPositions);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }
   
   /**
    * Use this constructor to go straight to the given end points.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredJointPositions desired joint positions. The array length should be equal to the number of joints.
    * @param weights the qp weights for the joint accelerations. If any index is set to NaN, that joint will use the controller default weight
    */
   public NeckTrajectoryMessage(double trajectoryTime, double[] desiredJointPositions, double[] weights)
   {
      jointspaceTrajectory = new JointspaceTrajectoryMessage(trajectoryTime, desiredJointPositions, weights);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Create a message using the given joint trajectory points.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param jointTrajectory1DListMessages joint trajectory points to be executed.
    */
   public NeckTrajectoryMessage(OneDoFJointTrajectoryMessage[] jointTrajectory1DListMessages)
   {
      jointspaceTrajectory = new JointspaceTrajectoryMessage(jointTrajectory1DListMessages);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Use this constructor to build a message with more than one trajectory point.
    * This constructor only allocates memory for the trajectories, you need to call {@link #setTrajectory1DMessage(int, OneDoFJointTrajectoryMessage)} for each joint afterwards.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param numberOfJoints number of joints that will be executing the message.
    */
   public NeckTrajectoryMessage(int numberOfJoints)
   {
      jointspaceTrajectory = new JointspaceTrajectoryMessage(numberOfJoints);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Use this constructor to build a message with more than one trajectory points.
    * This constructor only allocates memory for the trajectories, you need to call {@link #setTrajectoryPoint(int, int, double, double, double)} for each joint and trajectory point afterwards.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param numberOfJoints number of joints that will be executing the message.
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the controller.
    */
   public NeckTrajectoryMessage(int numberOfJoints, int numberOfTrajectoryPoints)
   {
      jointspaceTrajectory = new JointspaceTrajectoryMessage(numberOfJoints, numberOfTrajectoryPoints);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void setUniqueId(long uniqueId)
   {
      super.setUniqueId(uniqueId);
      if (jointspaceTrajectory != null)
         jointspaceTrajectory.setUniqueId(uniqueId);
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
   public boolean epsilonEquals(NeckTrajectoryMessage other, double epsilon)
   {
      if (!jointspaceTrajectory.epsilonEquals(other.jointspaceTrajectory, epsilon))
         return false;
      return true;
   }

   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateNeckTrajectoryMessage(this);
   }
}
