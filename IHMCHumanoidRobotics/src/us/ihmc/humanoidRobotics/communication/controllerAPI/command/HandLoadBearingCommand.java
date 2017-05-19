package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandLoadBearingMessage;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandLoadBearingCommand extends AbstractLoadBearingCommand<HandLoadBearingCommand, HandLoadBearingMessage>
{
   private RobotSide robotSide;

   private ArmTrajectoryCommand armTrajectoryCommand = null;

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   @Override
   public void set(HandLoadBearingCommand other)
   {
      super.set(other);
      robotSide = other.robotSide;
      if (other.armTrajectoryCommand != null)
      {
         if (armTrajectoryCommand == null)
            armTrajectoryCommand = new ArmTrajectoryCommand();
         armTrajectoryCommand.set(other.armTrajectoryCommand);
      }
   }

   @Override
   public void set(HandLoadBearingMessage message)
   {
      super.set(message);
      robotSide = message.robotSide;
      if (message.getArmTrajectoryMessage() != null)
      {
         if (armTrajectoryCommand == null)
            armTrajectoryCommand = new ArmTrajectoryCommand();
         armTrajectoryCommand.set(message.getArmTrajectoryMessage());
      }
   }

   public ArmTrajectoryCommand getArmTrajectoryCommand()
   {
      return armTrajectoryCommand;
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
