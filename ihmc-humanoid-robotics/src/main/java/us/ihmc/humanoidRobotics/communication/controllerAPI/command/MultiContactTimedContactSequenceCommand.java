package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.MultiContactTimedContactSequenceMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.List;
import java.util.Random;

public class MultiContactTimedContactSequenceCommand implements Command<MultiContactTimedContactSequenceCommand, MultiContactTimedContactSequenceMessage>
{
   private long sequenceId;

   private final SideDependentList<RecyclingArrayList<TimeIntervalCommand>> armContactIntervals = new SideDependentList<>(side -> new RecyclingArrayList<>(20, TimeIntervalCommand.class));
   private final SideDependentList<RecyclingArrayList<TimeIntervalCommand>> legContactIntervals = new SideDependentList<>(side -> new RecyclingArrayList<>(20, TimeIntervalCommand.class));

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

      for (RobotSide robotSide : RobotSide.values)
      {
         armContactIntervals.get(robotSide).clear();
         legContactIntervals.get(robotSide).clear();
      }
   }

   @Override
   public void set(MultiContactTimedContactSequenceCommand other)
   {
      clear();
      sequenceId = other.sequenceId;

      for (RobotSide robotSide : RobotSide.values)
      {
         for (int i = 0; i < other.armContactIntervals.get(robotSide).size(); i++)
         {
            armContactIntervals.get(robotSide).add().set(other.armContactIntervals.get(robotSide).get(i));
         }

         for (int i = 0; i < other.legContactIntervals.get(robotSide).size(); i++)
         {
            legContactIntervals.get(robotSide).add().set(other.legContactIntervals.get(robotSide).get(i));
         }
      }
   }

   @Override
   public void setFromMessage(MultiContactTimedContactSequenceMessage message)
   {
      clear();
      sequenceId = message.getSequenceId();

      for (int i = 0; i < message.getLeftArmContactIntervals().size(); i++)
      {
         armContactIntervals.get(RobotSide.LEFT).add().setFromMessage(message.getLeftArmContactIntervals().get(i));
      }

      for (int i = 0; i < message.getRightArmContactIntervals().size(); i++)
      {
         armContactIntervals.get(RobotSide.RIGHT).add().setFromMessage(message.getRightArmContactIntervals().get(i));
      }

      for (int i = 0; i < message.getLeftLegContactIntervals().size(); i++)
      {
         legContactIntervals.get(RobotSide.LEFT).add().setFromMessage(message.getLeftLegContactIntervals().get(i));
      }

      for (int i = 0; i < message.getRightLegContactIntervals().size(); i++)
      {
         legContactIntervals.get(RobotSide.RIGHT).add().setFromMessage(message.getRightLegContactIntervals().get(i));
      }
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }

   public void addTimeOffset(double timeOffsetToAdd)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         for (int i = 0; i < armContactIntervals.get(robotSide).size(); i++)
            armContactIntervals.get(robotSide).get(i).addTimeOffset(timeOffsetToAdd);
         for (int i = 0; i < legContactIntervals.get(robotSide).size(); i++)
            legContactIntervals.get(robotSide).get(i).addTimeOffset(timeOffsetToAdd);
      }
   }

   public List<TimeIntervalCommand> getArmContactIntervals(RobotSide robotSide)
   {
      return armContactIntervals.get(robotSide);
   }

   public List<TimeIntervalCommand> getLegContactIntervals(RobotSide robotSide)
   {
      return legContactIntervals.get(robotSide);
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
