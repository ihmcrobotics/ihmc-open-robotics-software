package us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI;

import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage;
import us.ihmc.communication.controllerAPI.command.Command;

public class WholeBodyTrajectoryToolboxConfigurationCommand
      implements Command<WholeBodyTrajectoryToolboxConfigurationCommand, WholeBodyTrajectoryToolboxConfigurationMessage>
{
   private long sequenceId;
   private int numberOfInitialGuesses = -1;
   private int maximumExpansionSize = -1;
   private boolean hasInitialConfiguration = false;
   private final KinematicsToolboxOutputStatus initialConfiguration = new KinematicsToolboxOutputStatus();

   public WholeBodyTrajectoryToolboxConfigurationCommand()
   {
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      numberOfInitialGuesses = -1;
      maximumExpansionSize = -1;
      hasInitialConfiguration = false;
   }

   @Override
   public void set(WholeBodyTrajectoryToolboxConfigurationCommand other)
   {
      clear();

      sequenceId = other.sequenceId;

      numberOfInitialGuesses = other.numberOfInitialGuesses;
      maximumExpansionSize = other.maximumExpansionSize;
      hasInitialConfiguration = other.hasInitialConfiguration;
      if (hasInitialConfiguration)
         initialConfiguration.set(other.initialConfiguration);
   }

   @Override
   public void setFromMessage(WholeBodyTrajectoryToolboxConfigurationMessage message)
   {
      clear();
      sequenceId = message.getSequenceId();
      numberOfInitialGuesses = message.getNumberOfInitialGuesses();
      maximumExpansionSize = message.getMaximumExpansionSize();
      hasInitialConfiguration = message.getInitialConfiguration() != null;
      if (hasInitialConfiguration)
         initialConfiguration.set(message.getInitialConfiguration());
   }

   public int getNumberOfInitialGuesses()
   {
      return numberOfInitialGuesses;
   }

   public int getMaximumExpansionSize()
   {
      return maximumExpansionSize;
   }

   public boolean hasInitialConfiguration()
   {
      return hasInitialConfiguration;
   }

   public KinematicsToolboxOutputStatus getInitialConfiguration()
   {
      return initialConfiguration;
   }

   @Override
   public Class<WholeBodyTrajectoryToolboxConfigurationMessage> getMessageClass()
   {
      return WholeBodyTrajectoryToolboxConfigurationMessage.class;
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
