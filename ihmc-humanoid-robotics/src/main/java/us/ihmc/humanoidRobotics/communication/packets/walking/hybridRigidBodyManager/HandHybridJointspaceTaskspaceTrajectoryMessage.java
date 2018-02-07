package us.ihmc.humanoidRobotics.communication.packets.walking.hybridRigidBodyManager;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.QueueableMessage;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.JointspaceTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.FrameBasedMessage;
import us.ihmc.humanoidRobotics.communication.packets.FrameInformation;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.robotics.robotSide.RobotSide;

@RosMessagePacket(documentation =
      "This message commands the controller to move the chest in both taskspace amd jointspace to the desired orientation and joint angles while going through the specified trajectory points.",
                  rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
                  topic = "/control/hybrid_hand_trajectory")
public class HandHybridJointspaceTaskspaceTrajectoryMessage extends Packet<HandHybridJointspaceTaskspaceTrajectoryMessage> implements VisualizablePacket, FrameBasedMessage
{
   @RosExportedField(documentation = "Specifies the side of the robot that will execute the trajectory.")
   public RobotSide robotSide;
   public HandTrajectoryMessage handTrajectoryMessage;
   public JointspaceTrajectoryMessage jointspaceTrajectoryMessage;
   public QueueableMessage queueingProperties = new QueueableMessage();

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public HandHybridJointspaceTaskspaceTrajectoryMessage()
   {
      super();
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Random constructor for unit testing this packet
    * @param random seed
    */
   public HandHybridJointspaceTaskspaceTrajectoryMessage(Random random)
   {
      this(new HandTrajectoryMessage(random), new JointspaceTrajectoryMessage(random));
   }

   /**
    * Clone constructor.
    * @param message to clone.
    */
   public HandHybridJointspaceTaskspaceTrajectoryMessage(HandHybridJointspaceTaskspaceTrajectoryMessage hybridJointspaceTaskspaceMessage)
   {
      robotSide = hybridJointspaceTaskspaceMessage.robotSide;
      handTrajectoryMessage = new HandTrajectoryMessage(hybridJointspaceTaskspaceMessage.getHandTrajectoryMessage());
      jointspaceTrajectoryMessage = new JointspaceTrajectoryMessage(hybridJointspaceTaskspaceMessage.jointspaceTrajectoryMessage);
      queueingProperties.set(hybridJointspaceTaskspaceMessage.queueingProperties);
      setUniqueId(hybridJointspaceTaskspaceMessage.getUniqueId());
   }

   /**
    * Typical constructor to use, pack the two taskspace and joint space commands.
    * If these messages conflict, the qp weights and gains will dictate the desireds
    * @param taskspaceTrajectoryMessage
    * @param jointspaceTrajectoryMessage
    */
   public HandHybridJointspaceTaskspaceTrajectoryMessage(HandTrajectoryMessage taskspaceTrajectoryMessage, JointspaceTrajectoryMessage jointspaceTrajectoryMessage)
   {
      if (!taskspaceTrajectoryMessage.getQueueingProperties().epsilonEquals(jointspaceTrajectoryMessage.getQueueingProperties(), 0.0))
         throw new IllegalArgumentException("The trajectory messages should have the same queueing properties.");

      robotSide = taskspaceTrajectoryMessage.robotSide;
      this.handTrajectoryMessage = new HandTrajectoryMessage(taskspaceTrajectoryMessage);
      this.jointspaceTrajectoryMessage = new JointspaceTrajectoryMessage(jointspaceTrajectoryMessage);
      queueingProperties.set(taskspaceTrajectoryMessage.getQueueingProperties());
      setUniqueId(taskspaceTrajectoryMessage.getUniqueId());
   }

   public HandTrajectoryMessage getHandTrajectoryMessage()
   {
      return handTrajectoryMessage;
   }

   public void setHandTrajectoryMessage(HandTrajectoryMessage handTrajectoryMessage)
   {
      this.handTrajectoryMessage = handTrajectoryMessage;
   }

   public JointspaceTrajectoryMessage getJointspaceTrajectoryMessage()
   {
      return jointspaceTrajectoryMessage;
   }

   public void setJointspaceTrajectoryMessage(JointspaceTrajectoryMessage jointspaceTrajectoryMessage)
   {
      this.jointspaceTrajectoryMessage = jointspaceTrajectoryMessage;
   }

   @Override
   public FrameInformation getFrameInformation()
   {
      return handTrajectoryMessage.getFrameInformation();
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public QueueableMessage getQueueingProperties()
   {
      return queueingProperties;
   }

   @Override
   public boolean epsilonEquals(HandHybridJointspaceTaskspaceTrajectoryMessage other, double epsilon)
   {
      if (!queueingProperties.epsilonEquals(other.queueingProperties, epsilon))
         return false;
      if (!handTrajectoryMessage.epsilonEquals(other.handTrajectoryMessage, epsilon))
         return false;
      if (!jointspaceTrajectoryMessage.epsilonEquals(other.jointspaceTrajectoryMessage, epsilon))
         return false;
      return false;
   }
}
