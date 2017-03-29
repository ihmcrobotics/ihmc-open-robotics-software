package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandLoadBearingMessage;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandLoadBearingCommand extends AbstractLoadBearingCommand<HandLoadBearingCommand, HandLoadBearingMessage>
{
   private RobotSide robotSide;

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   @Override
   public void set(HandLoadBearingCommand other)
   {
      super.set(other);
      robotSide = other.robotSide;
   }

   @Override
   public void set(HandLoadBearingMessage message)
   {
      super.set(message);
      robotSide = message.robotSide;
   }

   @Override
   public void clear()
   {
      super.clear();
      robotSide = null;
   }

   @Override
   public Class<HandLoadBearingMessage> getMessageClass()
   {
      return HandLoadBearingMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return robotSide != null && super.isCommandValid();
   }
}
