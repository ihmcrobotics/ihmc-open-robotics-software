package us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;

/**
 * A {@code FeedbackControlCommandList} gathers several command to be processed by the controller core.
 * <p>
 * This command can also contain {@code FeedbackControlCommandList}.
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
    * 
    * @param command the command to register. The command's reference is saved, no copy is done. Not modified.
    */
   public void addCommand(FeedbackControlCommand<?> command)
   {
      commandList.add(command);
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

   public int getNumberOfCommands()
   {
      return commandList.size();
   }

   /**
    * Adds all the commands contained in {@code other} to this.
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
    * @return {@link ControllerCoreCommandType#COMMAND_LIST}.
    */
   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.COMMAND_LIST;
   }
}
