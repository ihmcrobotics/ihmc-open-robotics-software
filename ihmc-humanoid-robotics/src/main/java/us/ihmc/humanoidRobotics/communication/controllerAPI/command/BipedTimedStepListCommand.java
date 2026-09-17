package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.BipedTimedStepListMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;

public class BipedTimedStepListCommand implements Command<BipedTimedStepListCommand, BipedTimedStepListMessage>
{
   private long sequenceId;
   private final RecyclingArrayList<BipedTimedStepCommand> commands = new RecyclingArrayList<>(BipedTimedStepCommand.class);

   public int getNumberOfSteps()
   {
      return commands.size();
   }

   public BipedTimedStepCommand getStep(int stepIdx)
   {
      return commands.get(stepIdx);
   }

   public void addStep(BipedTimedStepCommand step)
   {
      commands.add().set(step);
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      for (int i = 0; i < commands.size(); i++)
         commands.get(i).clear();
      commands.clear();
   }

   @Override
   public void setFromMessage(BipedTimedStepListMessage message)
   {
      clear();
      sequenceId = message.getSequenceId();
      for (int i = 0; i < message.getTimedStepList().size(); i++)
         commands.add().setFromMessage(message.getTimedStepList().get(i));
   }

   @Override
   public Class<BipedTimedStepListMessage> getMessageClass()
   {
      return BipedTimedStepListMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return getNumberOfSteps() > 0;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }

   @Override
   public void set(BipedTimedStepListCommand other)
   {
      clear();
      sequenceId = other.sequenceId;
      for (int i = 0; i < other.commands.size(); i++)
         commands.add().set(other.commands.get(i));
   }
}
