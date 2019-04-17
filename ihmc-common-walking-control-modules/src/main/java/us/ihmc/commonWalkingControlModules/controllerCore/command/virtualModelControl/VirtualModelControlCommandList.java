package us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyVirtualModelControlSolver;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;

/**
 * A {@link VirtualModelControlCommandList} gathers several commands to be submitted to the controller
 * core.
 * <p>
 * These commands eventually get processed by the {@link WholeBodyVirtualModelControlSolver} to compute
 * the desired joint accelerations and then desired joint torques.
 * </p>
 * <p>
 * When adding a {@link VirtualModelControlCommandList} to this list, it will unpack the given list such
 * that this list remains flat.
 * </p>
 * 
 * @author Robert Griffin
 *
 */
public class VirtualModelControlCommandList implements VirtualModelControlCommand<VirtualModelControlCommandList>
{
   /**
    * Internal storage of the commands.
    */
   private final List<VirtualModelControlCommand<?>> commandList = new ArrayList<>();

   /**
    * Creates an empty command list.
    */
   public VirtualModelControlCommandList()
   {
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
   public void addCommand(VirtualModelControlCommand<?> command)
   {
      if (command != null)
      {
         if (command instanceof VirtualModelControlCommandList)
            addCommandList((VirtualModelControlCommandList) command);
         else
            commandList.add(command);
      }
   }

   /**
    * Adds the commands contained in the given list to this list using the method
    * {@link #addCommand(VirtualModelControlCommand)} for each element.
    * 
    * @param commandList the list of commands to register. The command's reference is saved, no copy
    *           is done. Not modified.
    */
   public void addCommandList(VirtualModelControlCommandList commandList)
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
      commandList.clear();
   }

   /**
    * Retrieves a command given its index in the list.
    * 
    * @param commandIndex the index in the list.
    * @return the command positioned at the given index.
    */
   public VirtualModelControlCommand<?> getCommand(int commandIndex)
   {
      return commandList.get(commandIndex);
   }

   /**
    * Removes and returns the last command of this list.
    * 
    * @return the last command.
    * @deprecated this method should be removed and it is not really useful anyway.
    */
   public VirtualModelControlCommand<?> pollCommand()
   {
      if (commandList.isEmpty())
         return null;
      else
         return commandList.remove(getNumberOfCommands() - 1);
   }

   /**
    * Gets the number of {@link FeedbackControlCommand}s contained in this list.
    * 
    * @return the number of commands.
    */
   public int getNumberOfCommands()
   {
      return commandList.size();
   }

   /**
    * Tests if this list of {@link FeedbackControlCommand}s is empty.
    * 
    * @return {@code true} if this command is empty, {@code false} otherwise.
    */
   public boolean isCommandEmpty()
   {
      return commandList.isEmpty();
   }

   /**
    * Clears this and then adds all the commands contained in {@code other} to this.
    */
   @Override
   public void set(VirtualModelControlCommandList other)
   {
      clear();
      for (int i = 0; i < other.getNumberOfCommands(); i++)
         addCommand(other.getCommand(i));
   }

   /**
    * {@inheritDoc}
    * 
    * @return {@link ControllerCoreCommandType#COMMAND_LIST}.
    */
   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.COMMAND_LIST;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof VirtualModelControlCommandList)
      {
         VirtualModelControlCommandList other = (VirtualModelControlCommandList) object;

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

   @Override
   public String toString()
   {
      return "Nb of commands: " + getNumberOfCommands() + "\n" + commandList;
   }
}
