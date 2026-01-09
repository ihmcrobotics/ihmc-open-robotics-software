package us.ihmc.humanoidRobotics.communication.directionalControlToolboxAPI;

import controller_msgs.msg.dds.DirectionalControlInputMessage;
import us.ihmc.communication.controllerAPI.command.Command;

public class DirectionalControlInputCommand implements Command<DirectionalControlInputCommand, DirectionalControlInputMessage>
{
   private long sequenceId;
   private boolean walk;
   private double forward;
   private double right;
   private double clockwise;
   private boolean unitVelocities;

   @Override
   public void set(DirectionalControlInputCommand other)
   {
      clear();

      walk = other.walk;
      sequenceId = other.sequenceId;
      forward = other.forward;
      right = other.right;
      clockwise = other.clockwise;
      unitVelocities = other.unitVelocities;
   }

   @Override
   public void clear()
   {
      walk = false;
      sequenceId = 0;
      forward = 0.0;
      right = 0.0;
      clockwise = 0.0;
      unitVelocities = false;
   }

   @Override
   public void setFromMessage(DirectionalControlInputMessage message)
   {
      set(message);
   }

   public void set(DirectionalControlInputMessage message)
   {
      clear();
      walk = message.getWalk();
      sequenceId = message.getSequenceId();
      forward = message.getForward();
      right = message.getRight();
      clockwise = message.getClockwise();
      unitVelocities = message.getUnitVelocities();
   }

   @Override
   public Class<DirectionalControlInputMessage> getMessageClass()
   {
      return DirectionalControlInputMessage.class;
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

   public double getForward()
   {
      return forward;
   }

   public double getRight()
   {
      return right;
   }

   public double getClockwise()
   {
      return clockwise;
   }

   public boolean getUnitVelocities()
   {
      return unitVelocities;
   }

   public String toString()
   {
      return getClass().getSimpleName() + ": " + "{ forward  : " + String.valueOf(forward) + ", right    : " + String.valueOf(right) + ", clockwise: "
            + String.valueOf(clockwise) + "}";
   }
}
