package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.QueueableMessage;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.humanoidRobotics.communication.packets.JointspaceTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.OneDoFJointTrajectoryMessage;

public class SpineTrajectoryMessage extends Packet<SpineTrajectoryMessage>
{
   @RosExportedField(documentation = "Trajectories for each joint.")
   public JointspaceTrajectoryMessage jointspaceTrajectory;

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public SpineTrajectoryMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Clone constructor.
    * @param spineTrajectoryMessage message to clone.
    */
   public SpineTrajectoryMessage(SpineTrajectoryMessage spineTrajectoryMessage)
   {
      jointspaceTrajectory = new JointspaceTrajectoryMessage(spineTrajectoryMessage.jointspaceTrajectory);
      setUniqueId(spineTrajectoryMessage.getUniqueId());
   }

   public SpineTrajectoryMessage(JointspaceTrajectoryMessage jointspaceTrajectoryMessage)
   {
      jointspaceTrajectory = new JointspaceTrajectoryMessage(jointspaceTrajectoryMessage);
      setUniqueId(jointspaceTrajectoryMessage.getUniqueId());
   }

   /**
    * Use this constructor to build a message with more than one trajectory points.
    * This constructor only allocates memory for the trajectories, you need to call {@link #setTrajectoryPoint(int, int, double, double, double)} for each joint and trajectory point afterwards.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param numberOfJoints number of joints that will be executing the message.
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the controller.
    */
   public SpineTrajectoryMessage(int numberOfJoints, int numberOfTrajectoryPoints)
   {
      jointspaceTrajectory = new JointspaceTrajectoryMessage(numberOfJoints, numberOfTrajectoryPoints);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Use this constructor to build a message with more than one trajectory point.
    * This constructor only allocates memory for the trajectories, you need to call {@link #setTrajectory1DMessage(int, OneDoFJointTrajectoryMessage)} for each joint afterwards.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param numberOfJoints number of joints that will be executing the message.
    */
   public SpineTrajectoryMessage(int numberOfJoints)
   {
      jointspaceTrajectory = new JointspaceTrajectoryMessage(numberOfJoints);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Use this constructor to go straight to the given end points.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param jointDesireds desired joint positions. The array length should be equal to the number of joints.
    */
   public SpineTrajectoryMessage(double trajectoryTime, double[] jointDesireds)
   {
      jointspaceTrajectory = new JointspaceTrajectoryMessage(trajectoryTime, jointDesireds);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Use this constructor to go straight to the given end points.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param jointDesireds desired joint positions. The array length should be equal to the number of joints.
    * @param weights the qp weights for the joint accelerations. If any index is set to NaN, that joint will use the controller default weight
    */
   public SpineTrajectoryMessage(double trajectoryTime, double[] jointDesireds, double[] weights)
   {
      jointspaceTrajectory = new JointspaceTrajectoryMessage(trajectoryTime, jointDesireds, weights);
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
   public boolean epsilonEquals(SpineTrajectoryMessage other, double epsilon)
   {
      if (!jointspaceTrajectory.epsilonEquals(other.jointspaceTrajectory, epsilon))
         return false;
      return true;
   }

   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateSpineTrajectoryMessage(this);
   }
}
