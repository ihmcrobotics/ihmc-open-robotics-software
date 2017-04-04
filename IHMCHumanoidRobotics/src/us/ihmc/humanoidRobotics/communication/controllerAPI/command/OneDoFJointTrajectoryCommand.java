package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.OneDoFJointTrajectoryMessage;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1DList;

public class OneDoFJointTrajectoryCommand extends SimpleTrajectoryPoint1DList implements Command<OneDoFJointTrajectoryCommand, OneDoFJointTrajectoryMessage>
{
   private long commandId = Packet.VALID_MESSAGE_DEFAULT_ID;
   private double weight;

   public OneDoFJointTrajectoryCommand()
   {
   }

   public void clear()
   {
      super.clear();
      commandId = Packet.VALID_MESSAGE_DEFAULT_ID;
      setWeight(Double.NaN);
   }

   @Override
   public void set(OneDoFJointTrajectoryCommand other)
   {
      super.set(other);
      commandId = other.commandId;
      setWeight(other.getWeight());
   }

   @Override
   public void set(OneDoFJointTrajectoryMessage message)
   {
      message.getTrajectoryPoints(this);
      commandId = message.getUniqueId();
      setWeight(message.getWeight());
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

   public double getWeight()
   {
      return weight;
   }

   public void setWeight(double weight)
   {
      this.weight = weight;
   }
   
   @Override
   public boolean isCommandValid()
   {
      boolean numberOfTrajectoryPointsIsPositive = getNumberOfTrajectoryPoints() > 0;
      boolean weightIsValid = !Double.isInfinite(getWeight());
      if(Double.isFinite(weight))
      {
         weightIsValid &= weight >= 0;
      }
      return numberOfTrajectoryPointsIsPositive && weightIsValid;
   }
}
