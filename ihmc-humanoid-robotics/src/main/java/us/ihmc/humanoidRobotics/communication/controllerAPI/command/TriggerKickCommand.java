package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.TriggerKickMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.robotics.robotSide.RobotSide;

public class TriggerKickCommand implements Command<TriggerKickCommand, TriggerKickMessage>
{
   private long sequenceId;
   private boolean triggerKickRequested = false;
   private RobotSide robotSide;
   private double kickHeight;
   private double kickImpulse;
   private double kickTargetDistance;

   @Override
   public void clear()
   {
      sequenceId = 0;
      triggerKickRequested = false;
      robotSide = null;
      kickHeight = 0;
      kickImpulse = 0;
      kickTargetDistance = 0;
   }

   @Override
   public void set(TriggerKickCommand other)
   {
      sequenceId = other.sequenceId;
      triggerKickRequested = other.isTriggerKickRequested();
      robotSide = other.robotSide;
      kickHeight = other.kickHeight;
      kickImpulse = other.kickImpulse;
      kickTargetDistance = other.kickTargetDistance;
   }

   @Override
   public void setFromMessage(TriggerKickMessage message)
   {
      sequenceId = message.getSequenceId();
      triggerKickRequested = true;
      robotSide = RobotSide.fromByte(message.getRobotSide());
      kickHeight = message.getKickHeight();
      kickImpulse = message.getKickImpulse();
      kickTargetDistance = message.getKickTargetDistance();
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public double getKickHeight()
   {
      return kickHeight;
   }

   public void setKickHeight(double kickHeight)
   {
      this.kickHeight = kickHeight;
   }

   public double getKickImpulse()
   {
      return kickImpulse;
   }

   public boolean isTriggerKickRequested()
   {
      return triggerKickRequested;
   }

   public void setKickImpulse(double kickImpulse)
   {
      this.kickImpulse = kickImpulse;
   }

   public double getKickTargetDistance()
   {
      return kickTargetDistance;
   }

   public void setKickTargetDistance(double kickTargetDistance)
   {
      this.kickTargetDistance = kickTargetDistance;
   }

   @Override
   public Class<TriggerKickMessage> getMessageClass()
   {
      return TriggerKickMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      if (robotSide == null)
      {
         return false;
      }

      if (kickHeight < 0 || kickImpulse < 0 || kickTargetDistance < 0)
      {
         return false;
      }

      return true;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}