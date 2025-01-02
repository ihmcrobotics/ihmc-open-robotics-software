package us.ihmc.humanoidRobotics.communication.footstepStreamingToolboxAPI;

import toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.List;

public class FootstepStreamingToolboxInputCommand implements Command<FootstepStreamingToolboxInputCommand, FootstepStreamingToolboxInputMessage>
{
   private long sequenceId;
   private final RecyclingArrayList<FootstepStreamingToolboxTrackerCommand> inputs = new RecyclingArrayList<>(FootstepStreamingToolboxTrackerCommand::new);

   @Override
   public void clear()
   {
      sequenceId = 0;
      inputs.clear();
   }

   @Override
   public void set(FootstepStreamingToolboxInputCommand other)
   {
      sequenceId = other.sequenceId;
      inputs.clear();
      for (int i = 0; i < other.inputs.size(); i++)
         inputs.add().set(other.inputs.get(i));
   }

   @Override
   public void setFromMessage(FootstepStreamingToolboxInputMessage message)
   {
      sequenceId = message.getSequenceId();
      inputs.clear();
      for (int i = 0; i < message.getTrackers().size(); i++)
         inputs.add().setFromMessage(message.getTrackers().get(i));
   }

   public void removeInput(int index)
   {
      inputs.remove(index);
   }

   public void removeInput(FootstepStreamingToolboxTrackerCommand input)
   {
      inputs.remove(input);
   }

   public int getNumberOfInputs()
   {
      return inputs.size();
   }

   public FootstepStreamingToolboxTrackerCommand getInput(int index)
   {
      return inputs.get(index);
   }

   public List<FootstepStreamingToolboxTrackerCommand> getInputs()
   {
      return inputs;
   }

   public boolean hasInputFor(RobotSide side)
   {
      return getInputFor(side) != null;
   }

   public FootstepStreamingToolboxTrackerCommand getInputFor(RobotSide side)
   {
      for (int i = 0; i < inputs.size(); i++)
      {
         if (inputs.get(i).getSide() == side)
            return inputs.get(i);
      }
      return null;
   }

   @Override
   public Class<FootstepStreamingToolboxInputMessage> getMessageClass()
   {
      return FootstepStreamingToolboxInputMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      for (int i = 0; i < inputs.size(); i++)
      {
         if (!inputs.get(i).isCommandValid())
            return false;
      }

      return true;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
