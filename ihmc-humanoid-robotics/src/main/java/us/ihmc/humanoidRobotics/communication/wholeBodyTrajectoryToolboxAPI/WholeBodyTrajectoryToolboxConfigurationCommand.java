package us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.WholeBodyTrajectoryToolboxConfigurationMessage;

public class WholeBodyTrajectoryToolboxConfigurationCommand
      implements Command<WholeBodyTrajectoryToolboxConfigurationCommand, WholeBodyTrajectoryToolboxConfigurationMessage>
{
   public int numberOfInitialGuesses = -1;
   public int maximumExpansionSize = -1;
   public boolean hasInitialConfiguration = false;
   public final KinematicsToolboxOutputStatus initialConfiguration = new KinematicsToolboxOutputStatus();

   public WholeBodyTrajectoryToolboxConfigurationCommand()
   {
   }

   @Override
   public void clear()
   {
      numberOfInitialGuesses = -1;
      maximumExpansionSize = -1;
      hasInitialConfiguration = false;
   }

   @Override
   public void set(WholeBodyTrajectoryToolboxConfigurationCommand other)
   {
      clear();
      numberOfInitialGuesses = other.numberOfInitialGuesses;
      maximumExpansionSize = other.maximumExpansionSize;
      hasInitialConfiguration = other.hasInitialConfiguration;
      if (hasInitialConfiguration)
         initialConfiguration.set(other.initialConfiguration);
   }

   @Override
   public void set(WholeBodyTrajectoryToolboxConfigurationMessage message)
   {
      clear();
      numberOfInitialGuesses = message.getNumberOfInitialGuesses();
      maximumExpansionSize = message.getMaximumExpansionSize();
      hasInitialConfiguration = message.getInitialConfiguration() != null;
      if (hasInitialConfiguration)
         initialConfiguration.set(message.getInitialConfiguration());
   }

   @Override
   public Class<WholeBodyTrajectoryToolboxConfigurationMessage> getMessageClass()
   {
      return WholeBodyTrajectoryToolboxConfigurationMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return false;
   }
}
