package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import ihmc_common_msgs.msg.dds.Point2DMessage;
import us.ihmc.communication.controllerAPI.command.Command;

public class Point2DCommand implements Command<Point2DCommand, Point2DMessage>
{
   public void set()
   {

   }

   public void clear()
   {

   }

   @Override
   public void setFromMessage(Point2DMessage message)
   {

   }

   @Override
   public Class<Point2DMessage> getMessageClass()
   {
      return null;
   }

   @Override
   public boolean isCommandValid()
   {
      return false;
   }

   @Override
   public long getSequenceId()
   {
      return 0;
   }

   @Override
   public void set(Point2DCommand point2DCommand)
   {

   }
}
