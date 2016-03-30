package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.OneDoFJointTrajectoryMessage;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1DList;

public class OneDoFJointTrajectoryCommand extends SimpleTrajectoryPoint1DList implements Command<OneDoFJointTrajectoryCommand, OneDoFJointTrajectoryMessage>
{
   private long commandId = Packet.VALID_MESSAGE_DEFAULT_ID;

   public OneDoFJointTrajectoryCommand()
   {
   }

   public void clear()
   {
      super.clear();
      commandId = Packet.VALID_MESSAGE_DEFAULT_ID;
   }

   @Override
   public void set(OneDoFJointTrajectoryCommand other)
   {
      super.set(other);
      commandId = other.commandId;
   }

   @Override
   public void set(OneDoFJointTrajectoryMessage message)
   {
      message.getTrajectoryPoints(this);
      commandId = message.getUniqueId();
   }

   public void setCommandId(long commandId)
   {
      this.commandId = commandId;
   }

   public long getCommandId()
   {
      return commandId;
   }

   @Override
   public Class<OneDoFJointTrajectoryMessage> getMessageClass()
   {
      return OneDoFJointTrajectoryMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return getNumberOfTrajectoryPoints() > 0;
   }
}
