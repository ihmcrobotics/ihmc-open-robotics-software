package us.ihmc.humanoidRobotics.communication.packets.walking.hybridRigidBodyManager;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.AbstractSO3HybridJointSpaceTaskSpaceTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.NeckTrajectoryMessage;

@RosMessagePacket(documentation =
      "This message commands the controller to move the chest in both taskspace amd jointspace to the desired orientation and joint angles while going through the specified trajectory points.",
                  rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
                  topic = "/control/hybrid_head_trajectory")
public class HeadHybridJointspaceTaskspaceMessage extends AbstractSO3HybridJointSpaceTaskSpaceTrajectoryMessage<HeadHybridJointspaceTaskspaceMessage, HeadTrajectoryMessage, NeckTrajectoryMessage> implements VisualizablePacket
{
   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public HeadHybridJointspaceTaskspaceMessage()
   {
      super();
   }

   /**
    * Random constructor for unit testing this packet
    * @param random seed
    */
   public HeadHybridJointspaceTaskspaceMessage(Random random)
   {
      super(new HeadTrajectoryMessage(random), new NeckTrajectoryMessage(random));
   }

   /**
    * Clone constructor.
    * @param message to clone.
    */
   public HeadHybridJointspaceTaskspaceMessage(HeadHybridJointspaceTaskspaceMessage hybridJointspaceTaskspaceMessage)
   {
      super(hybridJointspaceTaskspaceMessage);
   }
   
   /**
    * Typical constructor to use, pack the two taskspace and joint space commands.
    * If these messages conflict, the qp weights and gains will dictate the desireds
    * @param headTrajectoryMessage
    * @param neckTrajectoryMessage
    */
   public HeadHybridJointspaceTaskspaceMessage(HeadTrajectoryMessage headTrajectoryMessage, NeckTrajectoryMessage neckTrajectoryMessage)
   {
      super(headTrajectoryMessage, neckTrajectoryMessage);
   }
}
