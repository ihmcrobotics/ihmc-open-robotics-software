package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.MultiContactTrajectoryMessage;
import controller_msgs.msg.dds.MultiContactTrajectorySequenceMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.idl.IDLSequence;

public class MultiContactTrajectorySequenceCommand implements Command<MultiContactTrajectorySequenceCommand, MultiContactTrajectorySequenceMessage>,
      EpsilonComparable<MultiContactTrajectorySequenceCommand>
{
   private long sequenceId;
   private final RecyclingArrayList<MultiContactTrajectoryCommand> trajectorySequence = new RecyclingArrayList<>(MultiContactTrajectoryCommand::new);

   @Override
   public void clear()
   {
      sequenceId = 0;
      trajectorySequence.clear();
   }

   @Override
   public void set(MultiContactTrajectorySequenceCommand other)
   {
      clear();
      sequenceId = other.sequenceId;
      for (int i = 0; i < other.trajectorySequence.size(); i++)
      {
         trajectorySequence.add().set(other.trajectorySequence.get(i));
      }
   }

   @Override
   public void setFromMessage(MultiContactTrajectorySequenceMessage message)
   {
      clear();
      sequenceId = message.getSequenceId();

      IDLSequence.Object<MultiContactTrajectoryMessage> trajectoryMessages = message.getTrajectorySequence();
      for (int i = 0; i < trajectoryMessages.size(); i++)
      {
         this.trajectorySequence.add().setFromMessage(trajectoryMessages.get(i));
      }
   }

   @Override
   public Class<MultiContactTrajectorySequenceMessage> getMessageClass()
   {
      return MultiContactTrajectorySequenceMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      for (int i = 0; i < trajectorySequence.size(); i++)
      {
         if (!trajectorySequence.get(i).isCommandValid())
         {
            return false;
         }
      }

      return true;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }

   @Override
   public boolean epsilonEquals(MultiContactTrajectorySequenceCommand other, double epsilon)
   {
      if (trajectorySequence.size() != other.trajectorySequence.size())
      {
         return false;
      }

      for (int i = 0; i < trajectorySequence.size(); i++)
      {
         if (!trajectorySequence.get(i).epsilonEquals(other.trajectorySequence.get(i), epsilon))
         {
            return false;
         }
      }

      return true;
   }

   public int size()
   {
      return trajectorySequence.size();
   }

   public MultiContactTrajectoryCommand getTrajectory(int index)
   {
      return trajectorySequence.get(index);
   }
}
