package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;

import java.util.ArrayList;
import java.util.List;

/**
 * A {@code MPCCommandList} gathers several commands to be submitted to the MPC
 * core.
 * <p>
 * These commands eventually get processed by the {@link us.ihmc.commonWalkingControlModules.modelPredictiveController.LinearMPCQPSolver} to compute
 * the desired motion function coefficients.
 * </p>
 * <p>
 * When adding a {@link MPCCommandList} to this list, it will unpack the given list such
 * that this list remains flat.
 * </p>
 */
public class MPCCommandList implements MPCCommand<MPCCommandList>
{
   private int commandId;
   /**
    * Internal storage of the commands.
    */
   private final List<MPCCommand<?>> commandList = new ArrayList<>();

   @Override
   public MPCCommandType getCommandType()
   {
      return MPCCommandType.LIST;
   }

   /**
    * Adds the given command to this list.
    * <p>
    * If the given command is a list, only its elements are added to this list not the list itself.
    * </p>
    *
    * @param command the command to register. The command's reference is saved, no copy is done. Not
    *           modified.
    */
   public void addCommand(MPCCommand<?> command)
   {
      if (command != null)
      {
         if (MPCCommandType.LIST == command.getCommandType())
            addCommandList((MPCCommandList) command);
         else
            commandList.add(command);
      }
   }

   /**
    * Adds the commands contained in the given list to this list using the method
    * {@link #addCommand(MPCCommand)} for each element.
    *
    * @param commandList the list of commands to register. The command's reference is saved, no copy is
    *           done. Not modified.
    */
   public void addCommandList(MPCCommandList commandList)
   {
      if (commandList == null)
         return;

      for (int i = 0; i < commandList.getNumberOfCommands(); i++)
         addCommand(commandList.getCommand(i));
   }

   /**
    * Clears all the commands contained in this list.
    */
   public void clear()
   {
      commandId = 0;
      commandList.clear();
   }

   /**
    * Retrieves a command given its index in the list.
    *
    * @param commandIndex the index in the list.
    * @return the command positioned at the given index.
    */
   public MPCCommand<?> getCommand(int commandIndex)
   {
      return commandList.get(commandIndex);
   }

   /**
    * Gets the number of {@link MPCCommand}s contained in this list.
    *
    * @return the number of commands.
    */
   public int getNumberOfCommands()
   {
      return commandList.size();
   }

   public void setCommandId(int id)
   {
      commandId = id;
   }

   public int getCommandId()
   {
      return commandId;
   }

   public void set(MPCCommandList other)
   {
      clear();
      setCommandId(other.getCommandId());
      for (int i = 0; i < other.getNumberOfCommands(); i++)
         addCommand(other.getCommand(i));
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof MPCCommandList)
      {
         MPCCommandList other = (MPCCommandList) object;

         if (commandId != other.commandId)
            return false;
         if (getNumberOfCommands() != other.getNumberOfCommands())
            return false;
         for (int commandIndex = 0; commandIndex < getNumberOfCommands(); commandIndex++)
         {
            if (!getCommand(commandIndex).equals(other.getCommand(commandIndex)))
               return false;
         }

         return true;
      }
      else
      {
         return false;
      }
   }
}
