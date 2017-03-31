package us.ihmc.humanoidRobotics.communication.packets.walking.hybridRigidBodyManager;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.AbstractSO3HybridJointSpaceTaskSpaceTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.SpineTrajectoryMessage;

@RosMessagePacket(documentation =
      "This message commands the controller to move the chest in both taskspace amd jointspace to the desired orientation and joint angles while going through the specified trajectory points.",
                  rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
                  topic = "/control/hybrid_chest_trajectory")
public class ChestHybridJointspaceTaskspaceMessage extends AbstractSO3HybridJointSpaceTaskSpaceTrajectoryMessage<ChestHybridJointspaceTaskspaceMessage, ChestTrajectoryMessage, SpineTrajectoryMessage> implements VisualizablePacket
{
   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public ChestHybridJointspaceTaskspaceMessage()
   {
      super();
   }

   /**
    * Random constructor for unit testing this packet
    * @param random seed
    */
   public ChestHybridJointspaceTaskspaceMessage(Random random)
   {
      super(new ChestTrajectoryMessage(random), new SpineTrajectoryMessage(random));
   }

   /**
    * Clone constructor.
    * @param message to clone.
    */
   public ChestHybridJointspaceTaskspaceMessage(ChestHybridJointspaceTaskspaceMessage chestHybridJointspaceTaskspaceMessage)
   {
      super(chestHybridJointspaceTaskspaceMessage);
   }
   
   /**
    * Typical constructor to use, pack the two taskspace and joint space commands.
    * If these messages conflict, the qp weights and gains will dictate the desireds
    * @param chestTrajectoryMessage
    * @param spineTrajectoryMessage
    */
   public ChestHybridJointspaceTaskspaceMessage(ChestTrajectoryMessage chestTrajectoryMessage, SpineTrajectoryMessage spineTrajectoryMessage)
   {
      super(chestTrajectoryMessage, spineTrajectoryMessage);
   }
}
