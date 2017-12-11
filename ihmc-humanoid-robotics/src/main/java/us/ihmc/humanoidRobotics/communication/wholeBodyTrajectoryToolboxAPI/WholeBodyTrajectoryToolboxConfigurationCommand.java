package us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxConfigurationMessage;

public class WholeBodyTrajectoryToolboxConfigurationCommand
      implements Command<WholeBodyTrajectoryToolboxConfigurationCommand, WholeBodyTrajectoryToolboxConfigurationMessage>
{
   private int numberOfInitialGuesses = -1;
   private int maximumExpansionSize = -1;
   private int trajectoryType = 0;
   private boolean hasInitialConfiguration = false;
   private final KinematicsToolboxOutputStatus initialConfiguration = new KinematicsToolboxOutputStatus();

   public WholeBodyTrajectoryToolboxConfigurationCommand()
   {
   }

   @Override
   public void clear()
   {
      numberOfInitialGuesses = -1;
      maximumExpansionSize = -1;
      trajectoryType = 0;
      hasInitialConfiguration = false;      
   }

   @Override
   public void set(WholeBodyTrajectoryToolboxConfigurationCommand other)
   {
      clear();
      numberOfInitialGuesses = other.numberOfInitialGuesses;
      maximumExpansionSize = other.maximumExpansionSize;
      trajectoryType = other.trajectoryType;
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
      trajectoryType = message.getTrajectoryType();
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
   
   public int getTrajectoryType()
   {
      return trajectoryType;
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
}
