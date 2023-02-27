package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import controller_msgs.msg.dds.ContinuousStepGeneratorInputMessage;
import us.ihmc.communication.controllerAPI.command.Command;

public class ContinuousStepGeneratorInputCommand implements Command<ContinuousStepGeneratorInputCommand, ContinuousStepGeneratorInputMessage>
{
   private long sequenceId;
   private boolean walk;
   private double forwardVelocity;
   private double lateralVelocity;
   private double turnVelocity;
   private boolean unitVelocities;

   @Override
   public void clear()
   {
      sequenceId = 0;
      walk = false;
      forwardVelocity = 0.0;
      lateralVelocity = 0.0;
      turnVelocity = 0.0;
   }

   @Override
   public void set(ContinuousStepGeneratorInputCommand other)
   {
      sequenceId = other.sequenceId;
      walk = other.walk;
      forwardVelocity = other.forwardVelocity;
      lateralVelocity = other.lateralVelocity;
      turnVelocity = other.turnVelocity;
      unitVelocities = other.unitVelocities;
   }

   @Override
   public void setFromMessage(ContinuousStepGeneratorInputMessage message)
   {
      sequenceId = message.getSequenceId();
      walk = message.getWalk();
      forwardVelocity = message.getForwardVelocity();
      lateralVelocity = message.getLateralVelocity();
      turnVelocity = message.getTurnVelocity();
      unitVelocities = message.getUnitVelocities();
   }

   @Override
   public Class<ContinuousStepGeneratorInputMessage> getMessageClass()
   {
      return ContinuousStepGeneratorInputMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }
   
   public void setWalk(boolean walk)
   {
      this.walk = walk;
   }

   public void setForwardVelocity(double forwardVelocity)
   {
      this.forwardVelocity = forwardVelocity;
   }

   public void setLateralVelocity(double lateralVelocity)
   {
      this.lateralVelocity = lateralVelocity;
   }

   public void setTurnVelocity(double turnVelocity)
   {
      this.turnVelocity = turnVelocity;
   }

   public void setUnitVelocities(boolean unitVelocities)
   {
      this.unitVelocities = unitVelocities;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }

   public boolean isWalk()
   {
      return walk;
   }

   public double getForwardVelocity()
   {
      return forwardVelocity;
   }

   public double getLateralVelocity()
   {
      return lateralVelocity;
   }

   public double getTurnVelocity()
   {
      return turnVelocity;
   }

   public boolean isUnitVelocities()
   {
      return unitVelocities;
   }
}
