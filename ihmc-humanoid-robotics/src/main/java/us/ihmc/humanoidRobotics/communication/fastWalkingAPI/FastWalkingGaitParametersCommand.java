package us.ihmc.humanoidRobotics.communication.fastWalkingAPI;

import controller_msgs.msg.dds.FastWalkingGaitParametersMessage;
import us.ihmc.communication.controllerAPI.command.Command;

public class FastWalkingGaitParametersCommand implements Command<FastWalkingGaitParametersCommand, FastWalkingGaitParametersMessage>
{
   private double swingHeight;
   private double swingDuration;
   private double doubleSupportFraction;

   @Override
   public void set(FastWalkingGaitParametersCommand other)
   {
      clear();

      swingHeight = other.swingHeight;
      swingDuration = other.swingDuration;
      doubleSupportFraction = other.doubleSupportFraction;
   }

   @Override
   public void clear()
   {
      swingDuration = 0.5;
      swingHeight = 0.1;
      doubleSupportFraction = 0.0;
   }

   @Override
   public void setFromMessage(FastWalkingGaitParametersMessage message)
   {
      set(message);
   }

   public void set(FastWalkingGaitParametersMessage message)
   {
      clear();
      swingHeight = message.getSwingHeight();
      swingDuration = message.getSwingDuration();
      doubleSupportFraction = message.getDoubleSupportFraction();
   }

   @Override
   public Class<FastWalkingGaitParametersMessage> getMessageClass()
   {
      return FastWalkingGaitParametersMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }

   @Override
   public long getSequenceId()
   {
      return -1;
   }

   public double getSwingHeight()
   {
      return swingHeight;
   }

   public double getDoubleSupportFraction()
   {
      return doubleSupportFraction;
   }

   public double getSwingDuration()
   {
      return swingDuration;
   }

   public String toString()
   {
      return getClass().getSimpleName() + ": " + "{ swing height  : " + String.valueOf(swingHeight) + ", swing duration    : " + String.valueOf(swingDuration)
             + ", double support fraction: " + String.valueOf(doubleSupportFraction) + "}";
   }
}
