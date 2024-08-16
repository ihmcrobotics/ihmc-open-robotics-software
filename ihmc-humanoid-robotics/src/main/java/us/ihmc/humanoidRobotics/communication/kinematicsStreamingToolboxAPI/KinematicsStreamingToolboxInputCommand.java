package us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI;

import toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.RigidBodyHashCodeResolver;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

import java.util.List;

public class KinematicsStreamingToolboxInputCommand implements Command<KinematicsStreamingToolboxInputCommand, KinematicsStreamingToolboxInputMessage>
{
   private long sequenceId;
   private long timestamp;
   private final RecyclingArrayList<KinematicsToolboxRigidBodyCommand> inputs = new RecyclingArrayList<>(KinematicsToolboxRigidBodyCommand::new);
   private boolean streamToController = false;
   private double streamInitialBlendDuration = -1.0;
   private double angularRateLimitation = -1.0;
   private double linearRateLimitation = -1.0;

   @Override
   public void clear()
   {
      sequenceId = 0;
      timestamp = 0;
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
      timestamp = other.timestamp;
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

   public void set(KinematicsStreamingToolboxInputMessage message,
                   RigidBodyHashCodeResolver rigidBodyHashCodeResolver,
                   ReferenceFrameHashCodeResolver referenceFrameResolver)
   {
      sequenceId = message.getSequenceId();
      timestamp = message.getTimestamp();
      inputs.clear();
      for (int i = 0; i < message.getInputs().size(); i++)
         inputs.add().set(message.getInputs().get(i), rigidBodyHashCodeResolver, referenceFrameResolver);
      streamToController = message.getStreamToController();
      streamInitialBlendDuration = message.getStreamInitialBlendDuration();
      angularRateLimitation = message.getAngularRateLimitation();
      linearRateLimitation = message.getLinearRateLimitation();
   }

   public void setTimestamp(long timestamp)
   {
      this.timestamp = timestamp;
   }

   public long getTimestamp()
   {
      return timestamp;
   }

   public void addInputs(List<KinematicsToolboxRigidBodyCommand> inputs)
   {
      for (int i = 0; i < inputs.size(); i++)
         this.inputs.add().set(inputs.get(i));
   }

   public void removeInput(int index)
   {
      inputs.remove(index);
   }

   public void removeInput(KinematicsToolboxRigidBodyCommand input)
   {
      inputs.remove(input);
   }

   public int getNumberOfInputs()
   {
      return inputs.size();
   }

   public KinematicsToolboxRigidBodyCommand getInput(int index)
   {
      return inputs.get(index);
   }

   public List<KinematicsToolboxRigidBodyCommand> getInputs()
   {
      return inputs;
   }

   public boolean hasInputFor(RigidBodyBasics endEffector)
   {
      return getInputFor(endEffector) != null;
   }

   public KinematicsToolboxRigidBodyCommand getInputFor(RigidBodyBasics endEffector)
   {
      for (int i = 0; i < inputs.size(); i++)
      {
         if (inputs.get(i).getEndEffector() == endEffector)
            return inputs.get(i);
      }
      return null;
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
