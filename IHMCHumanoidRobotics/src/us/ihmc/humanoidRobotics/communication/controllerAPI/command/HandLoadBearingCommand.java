package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandLoadBearingMessage;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandLoadBearingCommand extends AbstractLoadBearingCommand<HandLoadBearingCommand, HandLoadBearingMessage>
{
   private RobotSide robotSide;

   private boolean useJointspaceCommand = false;

   private ArmTrajectoryCommand armTrajectoryCommand = new ArmTrajectoryCommand();

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   @Override
   public void set(HandLoadBearingCommand other)
   {
      super.set(other);
      robotSide = other.robotSide;
      useJointspaceCommand = other.isUseJointspaceCommand();
      armTrajectoryCommand.set(other.getArmTrajectoryCommand());
   }

   @Override
   public void set(HandLoadBearingMessage message)
   {
      super.set(message);
      robotSide = message.robotSide;
      useJointspaceCommand = message.isUseJointspaceCommand();
      if (message.getArmTrajectoryMessage() != null)
      {
         armTrajectoryCommand.set(message.getArmTrajectoryMessage());
      }
   }

   public ArmTrajectoryCommand getArmTrajectoryCommand()
   {
      return armTrajectoryCommand;
   }

   public boolean isUseJointspaceCommand()
   {
      return useJointspaceCommand;
   }

   @Override
   public void clear()
   {
      super.clear();
      robotSide = null;
      useJointspaceCommand = false;
      armTrajectoryCommand.clear();
   }

   @Override
   public Class<HandLoadBearingMessage> getMessageClass()
   {
      return HandLoadBearingMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      boolean armTrajectoryValid = true;
      if (useJointspaceCommand && armTrajectoryCommand == null)
      {
         armTrajectoryValid = false;
      }
      else if (useJointspaceCommand)
      {
         armTrajectoryValid = armTrajectoryCommand.isCommandValid();
      }

      return armTrajectoryValid && robotSide != null && super.isCommandValid();
   }
}
