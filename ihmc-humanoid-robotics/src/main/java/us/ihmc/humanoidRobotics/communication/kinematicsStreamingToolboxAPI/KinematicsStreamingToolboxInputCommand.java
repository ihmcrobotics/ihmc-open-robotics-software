package us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI;

import controller_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.RigidBodyHashCodeResolver;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class KinematicsStreamingToolboxInputCommand implements Command<KinematicsStreamingToolboxInputCommand, KinematicsStreamingToolboxInputMessage>
{
   private long sequenceId;
   private final RecyclingArrayList<KinematicsToolboxRigidBodyCommand> inputs = new RecyclingArrayList<>(KinematicsToolboxRigidBodyCommand::new);
   private boolean streamToController = false;

   @Override
   public void clear()
   {
      sequenceId = 0;
      inputs.clear();
      streamToController = false;
   }

   @Override
   public void set(KinematicsStreamingToolboxInputCommand other)
   {
      sequenceId = other.sequenceId;
      inputs.clear();
      for (int i = 0; i < other.inputs.size(); i++)
         inputs.add().set(other.inputs.get(i));
      streamToController = other.streamToController;
   }

   @Override
   public void setFromMessage(KinematicsStreamingToolboxInputMessage message)
   {
      set(message, null, null);
   }

   public void set(KinematicsStreamingToolboxInputMessage message, RigidBodyHashCodeResolver rigidBodyHashCodeResolver,
                   ReferenceFrameHashCodeResolver referenceFrameResolver)
   {
      sequenceId = message.getSequenceId();
      inputs.clear();
      for (int i = 0; i < message.getInputs().size(); i++)
         inputs.add().set(message.getInputs().get(i), rigidBodyHashCodeResolver, referenceFrameResolver);
      streamToController = message.getStreamToController();
   }

   public int getNumberOfInputs()
   {
      return inputs.size();
   }

   public KinematicsToolboxRigidBodyCommand getInput(int index)
   {
      return inputs.get(index);
   }

   public boolean hasInputFor(RigidBodyBasics endEffector)
   {
      for (int i = 0; i < inputs.size(); i++)
      {
         if (inputs.get(i).getEndEffector() == endEffector)
            return true;
      }
      return false;
   }

   public boolean getStreamToController()
   {
      return streamToController;
   }

   @Override
   public Class<KinematicsStreamingToolboxInputMessage> getMessageClass()
   {
      return KinematicsStreamingToolboxInputMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
