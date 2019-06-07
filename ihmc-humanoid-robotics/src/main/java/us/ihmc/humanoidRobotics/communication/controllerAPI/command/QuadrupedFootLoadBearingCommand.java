package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.QuadrupedFootLoadBearingMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedFootLoadBearingCommand implements Command<QuadrupedFootLoadBearingCommand, QuadrupedFootLoadBearingMessage>
{
   private long sequenceId;

   /** Quadrant of the foot to be loaded */
   private RobotQuadrant robotQuadrant = null;

   @Override
   public void clear()
   {
      sequenceId = 0;
      robotQuadrant = null;
   }

   @Override
   public void setFromMessage(QuadrupedFootLoadBearingMessage message)
   {
      sequenceId = message.getSequenceId();
      robotQuadrant = RobotQuadrant.fromByte(message.getRobotQuadrant());
   }

   @Override
   public Class<QuadrupedFootLoadBearingMessage> getMessageClass()
   {
      return QuadrupedFootLoadBearingMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return robotQuadrant != null;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }

   public RobotQuadrant getRobotQuadrant()
   {
      return robotQuadrant;
   }

   @Override
   public void set(QuadrupedFootLoadBearingCommand other)
   {
      this.sequenceId = other.sequenceId;
      this.robotQuadrant = other.robotQuadrant;
   }
}
