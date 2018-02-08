package us.ihmc.humanoidRobotics.communication.packets.walking.hybridRigidBodyManager;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.SO3TrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.FrameBasedMessage;
import us.ihmc.humanoidRobotics.communication.packets.FrameInformation;
import us.ihmc.humanoidRobotics.communication.packets.JointspaceTrajectoryMessage;

@RosMessagePacket(documentation =
      "This message commands the controller to move the chest in both taskspace amd jointspace to the desired orientation and joint angles while going through the specified trajectory points.",
                  rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
                  topic = "/control/hybrid_head_trajectory")
public class HeadHybridJointspaceTaskspaceTrajectoryMessage extends Packet<HeadHybridJointspaceTaskspaceTrajectoryMessage>  implements VisualizablePacket, FrameBasedMessage
{
   public SO3TrajectoryMessage taskspaceTrajectoryMessage;
   public JointspaceTrajectoryMessage jointspaceTrajectoryMessage;

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public HeadHybridJointspaceTaskspaceTrajectoryMessage()
   {
      super();
   }

   /**
    * Random constructor for unit testing this packet
    * @param random seed
    */
   public HeadHybridJointspaceTaskspaceTrajectoryMessage(Random random)
   {
      this(new SO3TrajectoryMessage(random), new JointspaceTrajectoryMessage(random));
   }

   /**
    * Clone constructor.
    * @param message to clone.
    */
   public HeadHybridJointspaceTaskspaceTrajectoryMessage(HeadHybridJointspaceTaskspaceTrajectoryMessage hybridJointspaceTaskspaceMessage)
   {
      this(hybridJointspaceTaskspaceMessage.getTaskspaceTrajectoryMessage(), hybridJointspaceTaskspaceMessage.getJointspaceTrajectoryMessage());
      setUniqueId(hybridJointspaceTaskspaceMessage.getUniqueId());
   }

   /**
    * Typical constructor to use, pack the two taskspace and joint space commands.
    * If these messages conflict, the qp weights and gains will dictate the desireds
    * @param taskspaceTrajectoryMessage
    * @param jointspaceTrajectoryMessage
    */
   public HeadHybridJointspaceTaskspaceTrajectoryMessage(SO3TrajectoryMessage taskspaceTrajectoryMessage, JointspaceTrajectoryMessage jointspaceTrajectoryMessage)
   {
      this.taskspaceTrajectoryMessage = taskspaceTrajectoryMessage;
      this.jointspaceTrajectoryMessage = jointspaceTrajectoryMessage;
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public SO3TrajectoryMessage getTaskspaceTrajectoryMessage()
   {
      return taskspaceTrajectoryMessage;
   }

   public void setTaskspaceTrajectoryMessage(SO3TrajectoryMessage taskspaceTrajectoryMessage)
   {
      this.taskspaceTrajectoryMessage = taskspaceTrajectoryMessage;
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
      return taskspaceTrajectoryMessage.getFrameInformation();
   }

   @Override
   public boolean epsilonEquals(HeadHybridJointspaceTaskspaceTrajectoryMessage other, double epsilon)
   {
      if (!taskspaceTrajectoryMessage.epsilonEquals(other.taskspaceTrajectoryMessage, epsilon))
         return false;
      if (!jointspaceTrajectoryMessage.epsilonEquals(other.jointspaceTrajectoryMessage, epsilon))
         return false;
      return true;
   }
}
