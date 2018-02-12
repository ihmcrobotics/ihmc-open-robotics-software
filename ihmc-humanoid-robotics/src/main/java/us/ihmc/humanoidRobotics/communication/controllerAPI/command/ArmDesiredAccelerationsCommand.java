package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmDesiredAccelerationsMessage;
import us.ihmc.robotics.robotSide.RobotSide;

public class ArmDesiredAccelerationsCommand implements Command<ArmDesiredAccelerationsCommand, ArmDesiredAccelerationsMessage>
{
   private RobotSide robotSide;
   private final DesiredAccelerationsCommand desiredAccelerations = new DesiredAccelerationsCommand();

   public ArmDesiredAccelerationsCommand()
   {
      super();
   }

   @Override
   public void clear()
   {
      robotSide = null;
      desiredAccelerations.clear();
   }

   @Override
   public void set(ArmDesiredAccelerationsMessage message)
   {
      robotSide = message.getRobotSide();
      desiredAccelerations.set(message.desiredAccelerations);
   }

   @Override
   public void set(ArmDesiredAccelerationsCommand other)
   {
      robotSide = other.robotSide;
      desiredAccelerations.set(other.desiredAccelerations);
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public DesiredAccelerationsCommand getDesiredAccelerations()
   {
      return desiredAccelerations;
   }

   @Override
   public Class<ArmDesiredAccelerationsMessage> getMessageClass()
   {
      return ArmDesiredAccelerationsMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return robotSide != null && desiredAccelerations.isCommandValid();
   }

   @Override
   public boolean isDelayedExecutionSupported()
   {
      return desiredAccelerations.isDelayedExecutionSupported();
   }

   @Override
   public void setExecutionDelayTime(double delayTime)
   {
      desiredAccelerations.setExecutionDelayTime(delayTime);
   }

   @Override
   public void setExecutionTime(double adjustedExecutionTime)
   {
      desiredAccelerations.setExecutionTime(adjustedExecutionTime);
   }

   @Override
   public double getExecutionDelayTime()
   {
      return desiredAccelerations.getExecutionDelayTime();
   }

   @Override
   public double getExecutionTime()
   {
      return desiredAccelerations.getExecutionTime();
   }
}
