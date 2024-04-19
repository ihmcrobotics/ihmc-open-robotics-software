package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.KickDoorMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.robotics.robotSide.RobotSide;

public class KickDoorCommand implements Command<KickDoorCommand, KickDoorMessage>
{
   private long sequenceId;
   private boolean triggerKickRequested = false;
   private RobotSide robotSide;
   private double kickHeight;
   private double kickImpulse;
   private double kickTargetDistance;
   private double prekickWeightDistribution;

   @Override
   public void clear()
   {
      sequenceId = 0;
      triggerKickRequested = false;
      robotSide = null;
      kickHeight = 0;
      kickImpulse = 0;
      kickTargetDistance = 0;
      prekickWeightDistribution = 0.5;
   }

   @Override
   public void set(KickDoorCommand other)
   {
      sequenceId = other.sequenceId;
      triggerKickRequested = other.triggerKickRequested;
      robotSide = other.robotSide;
      kickHeight = other.kickHeight;
      kickImpulse = other.kickImpulse;
      kickTargetDistance = other.kickTargetDistance;
      prekickWeightDistribution = other.prekickWeightDistribution;
   }

   @Override
   public void setFromMessage(KickDoorMessage message)
   {
      sequenceId = message.getSequenceId();
      triggerKickRequested = true;
      robotSide = RobotSide.fromByte(message.getRobotSide());
      kickHeight = message.getKickHeight();
      kickImpulse = message.getKickImpulse();
      kickTargetDistance = message.getKickTargetDistance();
      prekickWeightDistribution = message.getPrekickWeightDistribution();
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

   public double getPrekickWeightDistribution()
   {
      return prekickWeightDistribution;
   }

   public void setPrekickWeightDistribution(double prekickWeightDistribution)
   {
      this.prekickWeightDistribution = prekickWeightDistribution;
   }

   @Override
   public Class<KickDoorMessage> getMessageClass()
   {
      return KickDoorMessage.class;
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

      if (prekickWeightDistribution < 0 || prekickWeightDistribution > 1)
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