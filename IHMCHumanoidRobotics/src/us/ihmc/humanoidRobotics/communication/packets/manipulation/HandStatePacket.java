package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandStatePacket extends Packet<HandStatePacket>
{
   public RobotSide robotSide;
   public boolean handClosed;
   public boolean closeThumb;

   public HandStatePacket()
   {
      // Empty constructor for deserialization
   }

   public HandStatePacket(RobotSide robotSide, boolean handClosed, boolean closeThumb)
   {
      this.robotSide = robotSide;
      this.handClosed = handClosed;
      this.closeThumb = closeThumb && handClosed;
   }

   public HandStatePacket(RobotSide robotSide, boolean handClosed)
   {
      this.robotSide = robotSide;
      this.handClosed = handClosed;
      this.closeThumb = false;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public boolean isHandClosed()
   {
      return handClosed;
   }

   public boolean closeThumb()
   {
      return closeThumb;
   }

   public String toString()
   {
      String side = robotSide.getSideNameFirstLetter() + " Hand";
      String thumbClose = "";
      String fingerClose = "";
      String open = "";
      if (this.closeThumb == true)
         thumbClose = "T&";
      if (this.handClosed == true)
         fingerClose = "F Closed";
      if ((this.handClosed == false) && (this.closeThumb == false))
         open = " Opened";

      return (side + " " + thumbClose + fingerClose + open);



   }

   public boolean equals(Object obj)
   {
      return ((obj instanceof HandStatePacket) && this.epsilonEquals((HandStatePacket) obj, 0));
   }

   @Override
   public boolean epsilonEquals(HandStatePacket other, double epsilon)
   {
      boolean ret = this.getRobotSide().equals(other.getRobotSide());
      ret &= (this.isHandClosed() == other.isHandClosed());
      ret &= (this.closeThumb() == other.closeThumb());

      return ret;
   }

   public HandStatePacket(Random random)
   {
      if (random.nextBoolean())
      {
         this.robotSide = RobotSide.LEFT;
      }
      else
      {
         this.robotSide = RobotSide.RIGHT;
      }

      this.handClosed = random.nextBoolean();
      this.closeThumb = random.nextBoolean() && handClosed;
   }
}
