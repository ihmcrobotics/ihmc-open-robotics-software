package us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyFeedbackController;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;

/**
 * A {@code FeedbackControlCommandList} gathers several commands to be submitted to the controller
 * core.
 * <p>
 * These commands eventually get processed by the adequate feedback controller in the
 * {@link WholeBodyFeedbackController}.
 * </p>
 * <p>
 * When adding a {@link FeedbackControlCommandList} to this list, it will unpack the given list such
 * that this list remains flat.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class FeedbackControlCommandList implements FeedbackControlCommand<FeedbackControlCommandList>
{
   private int commandId;
   /**
    * Internal storage of the commands.
    */
   private final List<FeedbackControlCommand<?>> commandList = new ArrayList<>();

   /**
    * Creates an empty command list.
    */
   public FeedbackControlCommandList()
   {
   }

   /**
    * Creates a new command list and initializes it to contain all the commands contained in
    * {@code other}.
    * 
    * @param other the other list from which all the commands added to this list. Not modified.
    */
   public FeedbackControlCommandList(FeedbackControlCommandList other)
   {
      addCommandList(other);
   }

   /**
    * Adds the given command to this list.
    * <p>
    * If the given command is a list, only its elements are added to this list not the list itself.
    * </p>
    * 
    * @param command the command to register. The command's reference is saved, no copy is done. Not
    *                modified.
    */
   public void addCommand(FeedbackControlCommand<?> command)
   {
      if (command == null)
         return;

      if (command instanceof FeedbackControlCommandList)
         addCommandList((FeedbackControlCommandList) command);
      else
         commandList.add(command);
   }

   /**
    * Adds the commands contained in the given list to this list using the method
    * {@link #addCommand(FeedbackControlCommand)} for each element.
    * 
    * @param commandList the list of commands to register. The command's reference is saved, no copy is
    *                    done. Not modified.
    */
   public void addCommandList(FeedbackControlCommandList commandList)
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
   public FeedbackControlCommand<?> getCommand(int commandIndex)
   {
      return commandList.get(commandIndex);
   }

   /**
    * Returns the index of the first occurrence of the specified command in this list, or {@code -1} if
    * this list does not contain it.
    * 
    * @param command the command to get the index of.
    * @return the index of the command, or {@code -1} if it could not be found.
    */
   public int indexOf(FeedbackControlCommand<?> command)
   {
      return commandList.indexOf(command);
   }

   /**
    * Removes the first occurrence of the specified command from this list, if it is present.
    * 
    * @param command the command to remove.
    */
   public void removeCommand(FeedbackControlCommand<?> command)
   {
      int indexOf = indexOf(command);

      if (indexOf == -1)
         return;
      removeCommand(indexOf);
   }

   /**
    * Removes the command at the specified position in this list and shifts the subsequent commands.
    * 
    * @param commandIndex the index of the command to be removed.
    */
   public void removeCommand(int commandIndex)
   {
      commandList.remove(commandIndex);
   }

   /**
    * Removes and returns the last command of this list.
    * 
    * @return the last command.
    * @deprecated this method should be removed and it is not really useful anyway.
    */
   public FeedbackControlCommand<?> pollCommand()
   {
      if (commandList.isEmpty())
         return null;
      else
         return commandList.remove(commandList.size() - 1);
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
   public void set(FeedbackControlCommandList other)
   {
      clear();
      setCommandId(other.getCommandId());
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
   public void setCommandId(int id)
   {
      commandId = id;
   }

   @Override
   public int getCommandId()
   {
      return commandId;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof FeedbackControlCommandList)
      {
         FeedbackControlCommandList other = (FeedbackControlCommandList) object;

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

   @Override
   public String toString()
   {
      return "Nb of commands: " + getNumberOfCommands() + "\n" + commandList;
   }
}
