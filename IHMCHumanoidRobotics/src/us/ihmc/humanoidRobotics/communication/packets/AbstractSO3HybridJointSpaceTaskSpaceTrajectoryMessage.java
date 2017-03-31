package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.QueueableMessage;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;

@RosMessagePacket(documentation = "Send whole body trajectories to the robot. A best effort is made to execute the trajectory while balance is kept.\n"
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule DOES apply to the fields of this message."
      + " If setting a field to null is not an option (going through IHMC ROS API), the user can use the latter rule to select the messages to be processed by the controller.",
      rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
      topic = "/control/whole_body_trajectory")

public class AbstractSO3HybridJointSpaceTaskSpaceTrajectoryMessage<
   M extends AbstractSO3HybridJointSpaceTaskSpaceTrajectoryMessage<M,TM,JM>, 
   TM extends AbstractSO3TrajectoryMessage<TM>, 
   JM extends AbstractJointspaceTrajectoryMessage<JM>
> extends QueueableMessage<M> implements VisualizablePacket
{
   @RosExportedField(documentation = "jointspace trajectory")
   public JM jointspaceTrajectoryMessage;

   @RosExportedField(documentation = "orientation trajectory")
   public TM taskspaceTrajectoryMessage;

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public AbstractSO3HybridJointSpaceTaskSpaceTrajectoryMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }
   
   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param chestHybridJointspaceTaskspaceMessage 
    */
   public AbstractSO3HybridJointSpaceTaskSpaceTrajectoryMessage(AbstractSO3HybridJointSpaceTaskSpaceTrajectoryMessage<M, TM, JM> hybridJointSpaceTaskSpaceTrajectoryMessage)
   {
      setUniqueId(hybridJointSpaceTaskSpaceTrajectoryMessage.uniqueId);
      this.taskspaceTrajectoryMessage = hybridJointSpaceTaskSpaceTrajectoryMessage.taskspaceTrajectoryMessage;
      this.jointspaceTrajectoryMessage = hybridJointSpaceTaskSpaceTrajectoryMessage.jointspaceTrajectoryMessage;
   }
   
   /**
    */
   public AbstractSO3HybridJointSpaceTaskSpaceTrajectoryMessage(TM taskspaceTrajectoryMessage, JM jointspaceTrajectoryMessage)
   {
      this.taskspaceTrajectoryMessage = taskspaceTrajectoryMessage;
      this.jointspaceTrajectoryMessage = jointspaceTrajectoryMessage;
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   
   public void setJointspaceTrajectoryMessage(JM jointspaceTrajectoryMessage)
   {
      if (jointspaceTrajectoryMessage.getUniqueId() == INVALID_MESSAGE_ID)
      {
         jointspaceTrajectoryMessage.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      }
      this.jointspaceTrajectoryMessage = jointspaceTrajectoryMessage;
   }
   
   public void setTaskspaceTrajectoryMessage(TM taskspaceTrajectoryMessage)
   {
      if (taskspaceTrajectoryMessage.getUniqueId() == INVALID_MESSAGE_ID)
      {
         taskspaceTrajectoryMessage.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      }
      this.taskspaceTrajectoryMessage = taskspaceTrajectoryMessage;
   }
   
   public JM getJointspaceTrajectoryMessage()
   {
      return jointspaceTrajectoryMessage;
   }

   public TM getTaskspaceTrajectoryMessage()
   {
      return taskspaceTrajectoryMessage;
   }

   public void clear()
   {
      clearJointspaceTrajectoryMessage();
      clearTaskspaceTrajectoryMessage();
   }

   public void clearJointspaceTrajectoryMessage()
   {
      jointspaceTrajectoryMessage = null;
   }

   public void clearTaskspaceTrajectoryMessage()
   {
      taskspaceTrajectoryMessage = null;
   }


   @Override
   public boolean epsilonEquals(M other, double epsilon)
   {
      if (jointspaceTrajectoryMessage == null && other.jointspaceTrajectoryMessage != null)
         return false;
      if (taskspaceTrajectoryMessage == null && other.taskspaceTrajectoryMessage != null)
         return false;
      if (jointspaceTrajectoryMessage != null && other.jointspaceTrajectoryMessage == null)
         return false;
      if (taskspaceTrajectoryMessage != null && other.taskspaceTrajectoryMessage == null)
         return false;
      if (!jointspaceTrajectoryMessage.epsilonEquals(other.jointspaceTrajectoryMessage, epsilon))
         return false;
      if (!taskspaceTrajectoryMessage.epsilonEquals(other.taskspaceTrajectoryMessage, epsilon))
         return false;

      return true;
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      String errorMessage = PacketValidityChecker.validatePacket(this, true);
      if (errorMessage != null)
         return errorMessage;

      if ((errorMessage = validateIfNeeded(jointspaceTrajectoryMessage)) != null)
         return errorMessage;
      if ((errorMessage = validateIfNeeded(taskspaceTrajectoryMessage)) != null)
         return errorMessage;

      return null;
   }

   private String validateIfNeeded(Packet<?> message)
   {
      String errorMessage = null;

      if (message != null && message.getUniqueId() != Packet.INVALID_MESSAGE_ID)
         errorMessage = message.validateMessage();

      return errorMessage;
   }
}
