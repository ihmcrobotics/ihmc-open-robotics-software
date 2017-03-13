package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmDesiredAccelerationsMessage;
import us.ihmc.robotics.robotSide.RobotSide;

public class ArmDesiredAccelerationsCommand extends DesiredAccelerationCommand<ArmDesiredAccelerationsCommand, ArmDesiredAccelerationsMessage>
{
   private RobotSide robotSide;

   public ArmDesiredAccelerationsCommand()
   {
      super();
   }

   @Override
   public void clear()
   {
      robotSide = null;
      super.clear();
   }

   @Override
   public void set(ArmDesiredAccelerationsMessage message)
   {
      robotSide = message.getRobotSide();
      super.set(message);
   }

   @Override
   public void set(ArmDesiredAccelerationsCommand other)
   {
      robotSide = other.robotSide;
      super.set(other);
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   @Override
   public Class<ArmDesiredAccelerationsMessage> getMessageClass()
   {
      return ArmDesiredAccelerationsMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return robotSide != null && super.isCommandValid();
   }
}
