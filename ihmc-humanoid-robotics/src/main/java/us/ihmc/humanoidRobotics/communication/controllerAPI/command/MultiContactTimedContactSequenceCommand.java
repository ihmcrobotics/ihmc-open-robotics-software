package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.MultiContactTimedContactSequenceMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;

import java.util.Random;

public class MultiContactTimedContactSequenceCommand implements Command<MultiContactTimedContactSequenceCommand, MultiContactTimedContactSequenceMessage>
{
   private long sequenceId;

   private final RecyclingArrayList<TimeIntervalCommand> leftArmContactIntervals = new RecyclingArrayList<>(20, TimeIntervalCommand.class);
   private final RecyclingArrayList<TimeIntervalCommand> rightArmContactIntervals = new RecyclingArrayList<>(20, TimeIntervalCommand.class);
   private final RecyclingArrayList<TimeIntervalCommand> leftLegContactIntervals = new RecyclingArrayList<>(20, TimeIntervalCommand.class);
   private final RecyclingArrayList<TimeIntervalCommand> rightLegContactIntervals = new RecyclingArrayList<>(20, TimeIntervalCommand.class);

   public MultiContactTimedContactSequenceCommand()
   {
      clear();
   }

   public MultiContactTimedContactSequenceCommand(Random random)
   {
      clear();
   }

   @Override
   public void clear()
   {
      sequenceId = 0;

      leftArmContactIntervals.clear();
      rightArmContactIntervals.clear();
      leftLegContactIntervals.clear();
      rightLegContactIntervals.clear();
   }

   @Override
   public void set(MultiContactTimedContactSequenceCommand other)
   {
      clear();
      sequenceId = other.sequenceId;

      for (int i = 0; i < other.leftArmContactIntervals.size(); i++)
      {
         leftArmContactIntervals.add().set(other.leftArmContactIntervals.get(i));
      }

      for (int i = 0; i < other.rightArmContactIntervals.size(); i++)
      {
         rightArmContactIntervals.add().set(other.rightArmContactIntervals.get(i));
      }

      for (int i = 0; i < other.leftLegContactIntervals.size(); i++)
      {
         leftLegContactIntervals.add().set(other.leftLegContactIntervals.get(i));
      }

      for (int i = 0; i < other.rightLegContactIntervals.size(); i++)
      {
         rightLegContactIntervals.add().set(other.rightLegContactIntervals.get(i));
      }
   }

   @Override
   public void setFromMessage(MultiContactTimedContactSequenceMessage message)
   {
      clear();
      sequenceId = message.getSequenceId();

      for (int i = 0; i < message.getLeftArmContactIntervals().size(); i++)
      {
         leftArmContactIntervals.add().setFromMessage(message.getLeftArmContactIntervals().get(i));
      }

      for (int i = 0; i < message.getRightArmContactIntervals().size(); i++)
      {
         rightArmContactIntervals.add().setFromMessage(message.getRightArmContactIntervals().get(i));
      }

      for (int i = 0; i < message.getLeftLegContactIntervals().size(); i++)
      {
         leftLegContactIntervals.add().setFromMessage(message.getLeftLegContactIntervals().get(i));
      }

      for (int i = 0; i < message.getRightLegContactIntervals().size(); i++)
      {
         rightLegContactIntervals.add().setFromMessage(message.getRightLegContactIntervals().get(i));
      }
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }

   public void addTimeOffset(double timeOffsetToAdd)
   {
      for (int i = 0; i < leftArmContactIntervals.size(); i++)
         leftArmContactIntervals.get(i).addTimeOffset(timeOffsetToAdd);
      for (int i = 0; i < rightArmContactIntervals.size(); i++)
         rightArmContactIntervals.get(i).addTimeOffset(timeOffsetToAdd);
      for (int i = 0; i < leftLegContactIntervals.size(); i++)
         leftLegContactIntervals.get(i).addTimeOffset(timeOffsetToAdd);
      for (int i = 0; i < rightLegContactIntervals.size(); i++)
         rightLegContactIntervals.get(i).addTimeOffset(timeOffsetToAdd);
   }

   @Override
   public Class<MultiContactTimedContactSequenceMessage> getMessageClass()
   {
      return MultiContactTimedContactSequenceMessage.class;
   }

   public void setSequenceId(long sequenceId)
   {
      this.sequenceId = sequenceId;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }

}
