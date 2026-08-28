package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.HighLevelStateMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;

public class HighLevelControllerStateCommand implements Command<HighLevelControllerStateCommand, HighLevelStateMessage>
{
   private long sequenceId;
   private HighLevelControllerName highLevelControllerName;
   private double trajectoryTime;

   @Override
   public void clear()
   {
      sequenceId = 0;
      highLevelControllerName = null;
      trajectoryTime = 0.0;
   }

   @Override
   public void set(HighLevelControllerStateCommand other)
   {
      sequenceId = other.sequenceId;
      highLevelControllerName = other.getHighLevelControllerName();
      trajectoryTime = other.trajectoryTime;
   }

   @Override
   public void setFromMessage(HighLevelStateMessage message)
   {
      sequenceId = message.getSequenceId();
      highLevelControllerName = HighLevelControllerName.fromByte(message.getHighLevelControllerName());
      trajectoryTime = message.getTrajectoryTime();
   }

   public void setHighLevelControllerName(HighLevelControllerName highLevelControllerName)
   {
      this.highLevelControllerName = highLevelControllerName;
   }

   public HighLevelControllerName getHighLevelControllerName()
   {
      return highLevelControllerName;
   }

   public void setTrajectoryTime(double trajectoryTime)
   {
      this.trajectoryTime = trajectoryTime;
   }

   public double getTrajectoryTime()
   {
      return trajectoryTime;
   }

   @Override
   public Class<HighLevelStateMessage> getMessageClass()
   {
      return HighLevelStateMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return highLevelControllerName != null;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
