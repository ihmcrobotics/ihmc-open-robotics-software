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
 *
 */
public class FeedbackControlCommandList implements FeedbackControlCommand<FeedbackControlCommandList>
{
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
    * Adds the given command to this list.
    * <p>
    * If the given command is a list, only its elements are added to this list not the list itself.
    * </p>
    * 
    * @param command the command to register. The command's reference is saved, no copy is done. Not
    *           modified.
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
    * @param commandList the list of commands to register. The command's reference is saved, no copy
    *           is done. Not modified.
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
    * Removes and returns the last command of this list.
    * 
    * @return the last command.
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
   public String toString()
   {
      return "Nb of commands: " + getNumberOfCommands() + "\n" + commandList;
   }
}
