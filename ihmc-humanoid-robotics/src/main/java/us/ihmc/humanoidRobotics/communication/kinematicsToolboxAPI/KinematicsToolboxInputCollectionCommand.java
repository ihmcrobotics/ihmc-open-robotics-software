package us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI;

import java.util.Objects;

import toolbox_msgs.msg.dds.KinematicsToolboxInputCollectionMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.robotModels.JointHashCodeResolver;
import us.ihmc.robotModels.RigidBodyHashCodeResolver;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class KinematicsToolboxInputCollectionCommand implements Command<KinematicsToolboxInputCollectionCommand, KinematicsToolboxInputCollectionMessage>
{
   private long sequenceId;
   private final RecyclingArrayList<KinematicsToolboxCenterOfMassCommand> centerOfMassInputs = new RecyclingArrayList<>(KinematicsToolboxCenterOfMassCommand::new);
   private final RecyclingArrayList<KinematicsToolboxRigidBodyCommand> rigidBodyInputs = new RecyclingArrayList<>(KinematicsToolboxRigidBodyCommand::new);
   private final RecyclingArrayList<KinematicsToolboxOneDoFJointCommand> jointInputs = new RecyclingArrayList<>(KinematicsToolboxOneDoFJointCommand::new);
   private boolean hasSupportRegionInput = false;
   private final KinematicsToolboxSupportRegionCommand supportRegionInput = new KinematicsToolboxSupportRegionCommand();

   @Override
   public void clear()
   {
      sequenceId = 0;
      centerOfMassInputs.clear();
      rigidBodyInputs.clear();
      jointInputs.clear();
      hasSupportRegionInput = false;
      supportRegionInput.clear();
   }

   @Override
   public void set(KinematicsToolboxInputCollectionCommand other)
   {
      clear();

      sequenceId = other.sequenceId;

      for (int i = 0; i < other.centerOfMassInputs.size(); i++)
      {
         centerOfMassInputs.add().set(other.centerOfMassInputs.get(i));
      }

      for (int i = 0; i < other.rigidBodyInputs.size(); i++)
      {
         rigidBodyInputs.add().set(other.rigidBodyInputs.get(i));
      }

      for (int i = 0; i < other.jointInputs.size(); i++)
      {
         jointInputs.add().set(other.jointInputs.get(i));
      }

      hasSupportRegionInput = other.hasSupportRegionInput;
      supportRegionInput.set(other.supportRegionInput);
   }

   @Override
   public void setFromMessage(KinematicsToolboxInputCollectionMessage message)
   {
      set(message, null, null, null);
   }

   public void set(KinematicsToolboxInputCollectionMessage message, RigidBodyHashCodeResolver rigidBodyHashCodeResolver,
                   ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver, JointHashCodeResolver jointHashCodeResolver)
   {
      Objects.requireNonNull(rigidBodyHashCodeResolver);
      Objects.requireNonNull(referenceFrameHashCodeResolver);
      Objects.requireNonNull(jointHashCodeResolver);

      clear();

      sequenceId = message.getSequenceId();

      for (int i = 0; i < message.getCenterOfMassInputs().size(); i++)
      {
         centerOfMassInputs.add().setFromMessage(message.getCenterOfMassInputs().get(i));
      }

      for (int i = 0; i < message.getRigidBodyInputs().size(); i++)
      {
         rigidBodyInputs.add().set(message.getRigidBodyInputs().get(i), rigidBodyHashCodeResolver, referenceFrameHashCodeResolver);
      }

      for (int i = 0; i < message.getJointInputs().size(); i++)
      {
         jointInputs.add().set(message.getJointInputs().get(i), jointHashCodeResolver);
      }

      hasSupportRegionInput = !message.getContactStateInput().isEmpty();
      if (hasSupportRegionInput)
         supportRegionInput.set(message.getContactStateInput().get(0), referenceFrameHashCodeResolver);
   }

   @Override
   public boolean isCommandValid()
   {
      for (int i = 0; i < centerOfMassInputs.size(); i++)
      {
         if (!centerOfMassInputs.get(i).isCommandValid())
            return false;
      }
      for (int i = 0; i < rigidBodyInputs.size(); i++)
      {
         if (!rigidBodyInputs.get(i).isCommandValid())
            return false;
      }
      for (int i = 0; i < jointInputs.size(); i++)
      {
         if (!jointInputs.get(i).isCommandValid())
            return false;
      }
      return true;
   }

   public RecyclingArrayList<KinematicsToolboxCenterOfMassCommand> getCenterOfMassInputs()
   {
      return centerOfMassInputs;
   }

   public RecyclingArrayList<KinematicsToolboxRigidBodyCommand> getRigidBodyInputs()
   {
      return rigidBodyInputs;
   }

   public RecyclingArrayList<KinematicsToolboxOneDoFJointCommand> getJointInputs()
   {
      return jointInputs;
   }

   public boolean hasSupportRegionInput()
   {
      return hasSupportRegionInput;
   }

   public KinematicsToolboxSupportRegionCommand getSupportRegionInput()
   {
      return supportRegionInput;
   }

   @Override
   public Class<KinematicsToolboxInputCollectionMessage> getMessageClass()
   {
      return KinematicsToolboxInputCollectionMessage.class;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
