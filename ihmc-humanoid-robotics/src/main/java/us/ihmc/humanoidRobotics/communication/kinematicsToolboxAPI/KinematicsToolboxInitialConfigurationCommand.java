package us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.JointHashCodeResolver;

public class KinematicsToolboxinitialConfigurationCommand
      implements Command<KinematicsToolboxinitialConfigurationCommand, KinematicsToolboxInitialConfigurationMessage>
{
   private long sequenceId;
   private final List<OneDoFJointBasics> joints = new ArrayList<>();
   private final TFloatArrayList initialJointAngles = new TFloatArrayList();

   @Override
   public void clear()
   {
      sequenceId = 0;
      joints.clear();
      initialJointAngles.reset();
   }

   @Override
   public void set(KinematicsToolboxinitialConfigurationCommand other)
   {
      clear();

      sequenceId = other.sequenceId;
      for (int i = 0; i < other.joints.size(); i++)
      {
         joints.add(other.joints.get(i));
         initialJointAngles.add(other.initialJointAngles.get(i));
      }
   }

   @Override
   public void setFromMessage(KinematicsToolboxInitialConfigurationMessage message)
   {
      set(message, null);
   }

   public void set(KinematicsToolboxInitialConfigurationMessage message, JointHashCodeResolver jointHashCodeResolver)
   {
      Objects.requireNonNull(jointHashCodeResolver);

      clear();

      sequenceId = message.getSequenceId();

      TIntArrayList messageHashCodes = message.getJointHashCodes();
      TFloatArrayList messageJointAngles = message.getJointAngles();
      for (int i = 0; i < messageHashCodes.size(); i++)
      {
         try
         {
            OneDoFJointBasics joint = jointHashCodeResolver.castAndGetJoint(messageHashCodes.get(i));
            joints.add(joint);
            initialJointAngles.add(messageJointAngles.get(i));
         }
         catch (RuntimeException e)
         {
            // unknown joint, skip.
         }
      }
   }

   public List<OneDoFJointBasics> getJoints()
   {
      return joints;
   }

   public TFloatArrayList getInitialJointAngles()
   {
      return initialJointAngles;
   }

   @Override
   public Class<KinematicsToolboxInitialConfigurationMessage> getMessageClass()
   {
      return KinematicsToolboxInitialConfigurationMessage.class;
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
