package us.ihmc.humanoidRobotics.communication.packets.walking.hybridRigidBodyManager;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.QueueableMessage;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.FrameBasedMessage;
import us.ihmc.humanoidRobotics.communication.packets.FrameInformation;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.SpineTrajectoryMessage;

@RosMessagePacket(documentation =
      "This message commands the controller to move the chest in both taskspace amd jointspace to the desired orientation and joint angles while going through the specified trajectory points.",
                  rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
                  topic = "/control/hybrid_chest_trajectory")
public class ChestHybridJointspaceTaskspaceTrajectoryMessage extends QueueableMessage<ChestHybridJointspaceTaskspaceTrajectoryMessage> implements VisualizablePacket, FrameBasedMessage
{

   public ChestTrajectoryMessage chestTrajectoryMessage;
   public SpineTrajectoryMessage spineTrajectoryMessage;

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public ChestHybridJointspaceTaskspaceTrajectoryMessage()
   {
      super();
   }

   /**
    * Random constructor for unit testing this packet
    * @param random seed
    */
   public ChestHybridJointspaceTaskspaceTrajectoryMessage(Random random)
   {
      this(new ChestTrajectoryMessage(random), new SpineTrajectoryMessage(random));
   }

   /**
    * Clone constructor.
    * @param message to clone.
    */
   public ChestHybridJointspaceTaskspaceTrajectoryMessage(ChestHybridJointspaceTaskspaceTrajectoryMessage chestHybridJointspaceTaskspaceMessage)
   {
      this(chestHybridJointspaceTaskspaceMessage.getChestTrajectoryMessage(), chestHybridJointspaceTaskspaceMessage.getSpineTrajectoryMessage());
      setExecutionDelayTime(chestHybridJointspaceTaskspaceMessage.getExecutionDelayTime());
   }

   /**
    * Typical constructor to use, pack the two taskspace and joint space commands.
    * If these messages conflict, the qp weights and gains will dictate the desireds
    * @param chestTrajectoryMessage
    * @param spineTrajectoryMessage
    */
   public ChestHybridJointspaceTaskspaceTrajectoryMessage(ChestTrajectoryMessage chestTrajectoryMessage, SpineTrajectoryMessage spineTrajectoryMessage)
   {
      this.chestTrajectoryMessage = chestTrajectoryMessage;
      this.spineTrajectoryMessage = spineTrajectoryMessage;
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public ChestTrajectoryMessage getChestTrajectoryMessage()
   {
      return chestTrajectoryMessage;
   }

   public void setChestTrajectoryMessage(ChestTrajectoryMessage chestTrajectoryMessage)
   {
      this.chestTrajectoryMessage = chestTrajectoryMessage;
   }

   public SpineTrajectoryMessage getSpineTrajectoryMessage()
   {
      return spineTrajectoryMessage;
   }

   public void setSpineTrajectoryMessage(SpineTrajectoryMessage spineTrajectoryMessage)
   {
      this.spineTrajectoryMessage = spineTrajectoryMessage;
   }

   @Override
   public FrameInformation getFrameInformation()
   {
      return chestTrajectoryMessage.getFrameInformation();
   }
}
