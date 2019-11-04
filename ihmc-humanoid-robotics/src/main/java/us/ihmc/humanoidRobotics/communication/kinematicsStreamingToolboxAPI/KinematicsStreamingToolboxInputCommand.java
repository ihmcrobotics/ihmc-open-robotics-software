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
   private double streamInitialBlendDuration = -1.0;
   private double angularRateLimitation = -1.0;
   private double linearRateLimitation = -1.0;

   @Override
   public void clear()
   {
      sequenceId = 0;
      inputs.clear();
      streamToController = false;
      streamInitialBlendDuration = -1.0;
      angularRateLimitation = -1.0;
      linearRateLimitation = -1.0;
   }

   @Override
   public void set(KinematicsStreamingToolboxInputCommand other)
   {
      sequenceId = other.sequenceId;
      inputs.clear();
      for (int i = 0; i < other.inputs.size(); i++)
         inputs.add().set(other.inputs.get(i));
      streamToController = other.streamToController;
      streamInitialBlendDuration = other.streamInitialBlendDuration;
      angularRateLimitation = other.angularRateLimitation;
      linearRateLimitation = other.linearRateLimitation;
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
      streamInitialBlendDuration = message.getStreamInitialBlendDuration();
      angularRateLimitation = message.getAngularRateLimitation();
      linearRateLimitation = message.getLinearRateLimitation();
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

   public double getStreamInitialBlendDuration()
   {
      return streamInitialBlendDuration;
   }

   public double getAngularRateLimitation()
   {
      return angularRateLimitation;
   }

   public double getLinearRateLimitation()
   {
      return linearRateLimitation;
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
