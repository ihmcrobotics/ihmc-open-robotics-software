package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.ArmDesiredAccelerationsMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.robotics.robotSide.RobotSide;

public class ArmDesiredAccelerationsCommand implements Command<ArmDesiredAccelerationsCommand, ArmDesiredAccelerationsMessage>
{
   private long sequenceId;
   private RobotSide robotSide;
   private final DesiredAccelerationsCommand desiredAccelerations = new DesiredAccelerationsCommand();

   public ArmDesiredAccelerationsCommand()
   {
      super();
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      robotSide = null;
      desiredAccelerations.clear();
   }

   @Override
   public void setFromMessage(ArmDesiredAccelerationsMessage message)
   {
      sequenceId = message.getSequenceId();
      robotSide = RobotSide.fromByte(message.getRobotSide());
      desiredAccelerations.setFromMessage(message.getDesiredAccelerations());
   }

   @Override
   public void set(ArmDesiredAccelerationsCommand other)
   {
      sequenceId = other.sequenceId;
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

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
