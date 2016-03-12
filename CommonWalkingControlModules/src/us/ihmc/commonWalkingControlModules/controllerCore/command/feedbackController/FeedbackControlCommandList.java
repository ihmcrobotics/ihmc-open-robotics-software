package us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController;

import java.util.ArrayList;
import java.util.List;

public class FeedbackControlCommandList implements FeedbackControlCommand<FeedbackControlCommandList>
{
   private final List<FeedbackControlCommand<?>> commandList = new ArrayList<>();

   public FeedbackControlCommandList()
   {
   }

   public void addCommand(FeedbackControlCommand<?> command)
   {
      commandList.add(command);
   }

   public void clear()
   {
      commandList.clear();
   }

   public FeedbackControlCommand<?> getCommand(int commandIndex)
   {
      return commandList.get(commandIndex);
   }

   public int getNumberOfCommands()
   {
      return commandList.size();
   }

   @Override
   public void set(FeedbackControlCommandList other)
   {
      clear();
      for (int i = 0; i < other.getNumberOfCommands(); i++)
         addCommand(other.getCommand(i));
   }

   @Override
   public FeedbackControlCommandType getCommandType()
   {
      return FeedbackControlCommandType.COMMAND_LIST;
   }
}
