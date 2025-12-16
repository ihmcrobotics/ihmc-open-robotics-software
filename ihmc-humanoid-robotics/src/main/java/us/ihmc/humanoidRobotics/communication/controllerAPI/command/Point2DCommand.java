package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import ihmc_common_msgs.msg.dds.Point2DMessage;
import us.ihmc.communication.controllerAPI.command.Command;

public class Point2DCommand implements Command<Point2DCommand, Point2DMessage>
{
   // TODO do we need sequence ID?
   private double x;
   private double y;

   @Override
   public void set(Point2DCommand other)
   {
      clear();

      x = other.getX();
      y = other.getY();
   }

   public void clear()
   {
      x = 0.0;
      y = 0.0;
   }

   @Override
   public void setFromMessage(Point2DMessage message)
   {
      set(message);
   }

   public void set(Point2DMessage message)
   {
      clear();

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
      return 0;
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
