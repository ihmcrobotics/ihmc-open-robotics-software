package us.ihmc.humanoidRobotics.communication.velocityBasedWalkingAPI;

import controller_msgs.msg.dds.VelocityBasedWalkingInputMessage;
import us.ihmc.communication.controllerAPI.command.Command;

public class VelocityBasedWalkingInputCommand implements Command<VelocityBasedWalkingInputCommand, VelocityBasedWalkingInputMessage>
{
   private long sequenceId;
   private boolean walk;
   private boolean jump;
   private double forwardVelocity;
   private double lateralVelocity;
   private double turnVelocity;
   private boolean areVelocitiesNormalized;

   @Override
   public void set(VelocityBasedWalkingInputCommand other)
   {
      clear();

      walk = other.walk;
      jump = other.jump;
      sequenceId = other.sequenceId;
      forwardVelocity = other.forwardVelocity;
      lateralVelocity = other.lateralVelocity;
      turnVelocity = other.turnVelocity;
      areVelocitiesNormalized = other.areVelocitiesNormalized;
   }

   @Override
   public void clear()
   {
      walk = false;
      jump = false;
      sequenceId = 0;
      forwardVelocity = 0.0;
      lateralVelocity = 0.0;
      turnVelocity = 0.0;
      areVelocitiesNormalized = false;
   }

   @Override
   public void setFromMessage(VelocityBasedWalkingInputMessage message)
   {
      set(message);
   }

   public void set(VelocityBasedWalkingInputMessage message)
   {
      clear();
      walk = message.getWalk();
      jump = message.getJump();
      sequenceId = message.getSequenceId();
      forwardVelocity = message.getForwardVelocity();
      lateralVelocity = message.getLateralVelocity();
      turnVelocity = message.getTurnVelocity();
      areVelocitiesNormalized = message.getAreVelocitiesNormalized();
   }

   @Override
   public Class<VelocityBasedWalkingInputMessage> getMessageClass()
   {
      return VelocityBasedWalkingInputMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }

   public boolean getWalk()
   {
      return walk;
   }

   public boolean getJump()
   {
      return jump;
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

   public boolean getAreVelocitiesNormalized()
   {
      return areVelocitiesNormalized;
   }

   public String toString()
   {
      return getClass().getSimpleName() + ": " + "{ forward  : " + String.valueOf(forwardVelocity) + ", lateral    : " + String.valueOf(lateralVelocity) + ", turn: "
             + String.valueOf(turnVelocity) + "}";
   }
}
