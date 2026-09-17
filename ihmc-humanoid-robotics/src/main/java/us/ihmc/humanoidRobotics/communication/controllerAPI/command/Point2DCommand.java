package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import ihmc_common_msgs.Point2DMessage;
import us.ihmc.communication.controllerAPI.command.Command;

public class Point2DCommand implements Command<Point2DCommand, Point2DMessage>
{
   private long sequenceId;
   private double x = Double.NaN;
   private double y = Double.NaN;

   @Override
   public void set(Point2DCommand other)
   {
      clear();

      sequenceId = other.sequenceId;
      x = other.getX();
      y = other.getY();
   }

   public void clear()
   {
      sequenceId = 0;
      x = Double.NaN;
      y = Double.NaN;
   }

   @Override
   public void setFromMessage(Point2DMessage message)
   {
      set(message);
   }

   public void set(Point2DMessage message)
   {
      clear();

      sequenceId = message.getSequenceId();
      x = message.getX();
      y = message.getY();
   }

   @Override
   public Class<Point2DMessage> getMessageClass()
   {
      return Point2DMessage.class;
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

   public double getX()
   {
      return x;
   }

   public double getY()
   {
      return y;
   }

   public String toString()
   {
      return getClass().getSimpleName() + ": " + "{ x  : " + String.valueOf(x) + ", y    : " + String.valueOf(y)  + "}";
   }
}
