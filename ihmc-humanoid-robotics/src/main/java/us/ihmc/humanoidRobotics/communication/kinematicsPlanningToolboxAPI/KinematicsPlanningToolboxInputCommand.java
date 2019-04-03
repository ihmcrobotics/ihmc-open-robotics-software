package us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI;

import java.util.Map;

import controller_msgs.msg.dds.KinematicsPlanningToolboxInputMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxConfigurationCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class KinematicsPlanningToolboxInputCommand implements Command<KinematicsPlanningToolboxInputCommand, KinematicsPlanningToolboxInputMessage>,
      KinematicsPlanningToolboxAPI<KinematicsPlanningToolboxInputMessage>
{
   private long sequenceId;
   private final RecyclingArrayList<KinematicsPlanningToolboxRigidBodyCommand> rigidBodyCommands = new RecyclingArrayList<>(KinematicsPlanningToolboxRigidBodyCommand.class);
   private final KinematicsPlanningToolboxCenterOfMassCommand centerOfMassCommand = new KinematicsPlanningToolboxCenterOfMassCommand();
   private final KinematicsToolboxConfigurationCommand kinematicsConfigurationCommand = new KinematicsToolboxConfigurationCommand();

   @Override
   public void set(KinematicsPlanningToolboxInputCommand other)
   {
      clear();

      sequenceId = other.sequenceId;

      for (int i = 0; i < other.getRigidBodyCommands().size(); i++)
         rigidBodyCommands.add().set(other.getRigidBodyCommands().get(i));
      centerOfMassCommand.set(other.getCenterOfMassCommand());
      kinematicsConfigurationCommand.set(other.getKinematicsConfigurationCommand());
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      rigidBodyCommands.clear();
      centerOfMassCommand.clear();
      kinematicsConfigurationCommand.clear();
   }

   @Override
   public void setFromMessage(KinematicsPlanningToolboxInputMessage message)
   {
      set(message, null, null);
   }

   public void set(KinematicsPlanningToolboxInputMessage message, Map<Integer, RigidBodyBasics> rigidBodyHashMap,
                   ReferenceFrameHashCodeResolver referenceFrameResolver)
   {
      clear();
      sequenceId = message.getSequenceId();
      for (int i = 0; i < message.getRigidBodyMessages().size(); i++)
         rigidBodyCommands.add().set(message.getRigidBodyMessages().get(i), rigidBodyHashMap, referenceFrameResolver);
      centerOfMassCommand.setFromMessage(message.getCenterOfMassMessage());
      kinematicsConfigurationCommand.setFromMessage(message.getKinematicsConfigurationMessage());
   }

   @Override
   public Class<KinematicsPlanningToolboxInputMessage> getMessageClass()
   {
      return KinematicsPlanningToolboxInputMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }

   public RecyclingArrayList<KinematicsPlanningToolboxRigidBodyCommand> getRigidBodyCommands()
   {
      return rigidBodyCommands;
   }

   public KinematicsPlanningToolboxCenterOfMassCommand getCenterOfMassCommand()
   {
      return centerOfMassCommand;
   }

   public KinematicsToolboxConfigurationCommand getKinematicsConfigurationCommand()
   {
      return kinematicsConfigurationCommand;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
