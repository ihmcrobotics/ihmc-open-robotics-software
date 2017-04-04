package us.ihmc.humanoidRobotics.communication.packets.walking.hybridRigidBodyManager;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.QueueableMessage;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.FrameBasedMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

@RosMessagePacket(documentation =
      "This message commands the controller to move the chest in both taskspace amd jointspace to the desired orientation and joint angles while going through the specified trajectory points.",
                  rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
                  topic = "/control/hybrid_hand_trajectory")
public class HandHybridJointspaceTaskspaceTrajectoryMessage extends QueueableMessage<HandHybridJointspaceTaskspaceTrajectoryMessage> implements VisualizablePacket, FrameBasedMessage
{
   
   private HandTrajectoryMessage handTrajectoryMessage;
   private ArmTrajectoryMessage armTrajectoryMessage;
   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public HandHybridJointspaceTaskspaceTrajectoryMessage()
   {
      super();
   }

   /**
    * Random constructor for unit testing this packet
    * @param random seed
    */
   public HandHybridJointspaceTaskspaceTrajectoryMessage(Random random)
   {
      this(new HandTrajectoryMessage(random), new ArmTrajectoryMessage(random));
   }

   /**
    * Clone constructor.
    * @param message to clone.
    */
   public HandHybridJointspaceTaskspaceTrajectoryMessage(HandHybridJointspaceTaskspaceTrajectoryMessage hybridJointspaceTaskspaceMessage)
   {
      this(hybridJointspaceTaskspaceMessage.getHandTrajectoryMessage(), hybridJointspaceTaskspaceMessage.getArmTrajectoryMessage());
   }
   
   /**
    * Typical constructor to use, pack the two taskspace and joint space commands.
    * If these messages conflict, the qp weights and gains will dictate the desireds
    * @param taskspaceTrajectoryMessage
    * @param jointspaceTrajectoryMessage
    */
   public HandHybridJointspaceTaskspaceTrajectoryMessage(HandTrajectoryMessage taskspaceTrajectoryMessage, ArmTrajectoryMessage jointspaceTrajectoryMessage)
   {
      handTrajectoryMessage = new HandTrajectoryMessage(taskspaceTrajectoryMessage);
      armTrajectoryMessage = new ArmTrajectoryMessage(jointspaceTrajectoryMessage);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public HandTrajectoryMessage getHandTrajectoryMessage()
   {
      return handTrajectoryMessage;
   }

   public void setHandTrajectoryMessage(HandTrajectoryMessage handTrajectoryMessage)
   {
      this.handTrajectoryMessage = handTrajectoryMessage;
   }

   public ArmTrajectoryMessage getArmTrajectoryMessage()
   {
      return armTrajectoryMessage;
   }

   public void setArmTrajectoryMessage(ArmTrajectoryMessage armTrajectoryMessage)
   {
      this.armTrajectoryMessage = armTrajectoryMessage;
   }
   
   @Override
   public long getTrajectoryReferenceFrameId()
   {
      return handTrajectoryMessage.getTrajectoryReferenceFrameId();
   }

   @Override
   public long getDataReferenceFrameId()
   {
      return handTrajectoryMessage.getDataReferenceFrameId();
   }

   @Override
   public void setTrajectoryReferenceFrameId(long trajedtoryReferenceFrameId)
   {
      handTrajectoryMessage.setTrajectoryReferenceFrameId(trajedtoryReferenceFrameId);
   }

   @Override
   public void setTrajectoryReferenceFrameId(ReferenceFrame trajectoryReferenceFrame)
   {
      handTrajectoryMessage.setTrajectoryReferenceFrameId(trajectoryReferenceFrame);
   }

   @Override
   public void setDataReferenceFrameId(long expressedInReferenceFrameId)
   {
      handTrajectoryMessage.setDataReferenceFrameId(expressedInReferenceFrameId);
   }

   @Override
   public void setDataReferenceFrameId(ReferenceFrame expressedInReferenceFrame)
   {
      handTrajectoryMessage.setDataReferenceFrameId(expressedInReferenceFrame);
   }
}
