package us.ihmc.communication.packets.walking;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.robotSide.RobotSide;

public class ThighStatePacket extends Packet<ThighStatePacket>
{
   public RobotSide robotSide;

   public boolean loadBearing;

   public ThighStatePacket()
   {
      // Empty constructor for deserialization
   }

   public ThighStatePacket(RobotSide robotSide, boolean loadBearing)
   {
      this.robotSide = robotSide;
      this.loadBearing = loadBearing;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public boolean isLoadBearing()
   {
      return loadBearing;
   }

   public void setLoadBearing(boolean loadBearing)
   {
      this.loadBearing = loadBearing;
   }

   public String toString()
   {
      return robotSide.getSideNameFirstLetter() + " Thigh Load Bearing";
   }

   public boolean equals(Object obj)
   {
      return ((obj instanceof ThighStatePacket) && this.epsilonEquals((ThighStatePacket) obj, 0));
   }

   @Override
   public boolean epsilonEquals(ThighStatePacket other, double epsilon)
   {
      boolean ret = (this.getRobotSide() == other.getRobotSide());
      ret &= (this.isLoadBearing() == other.isLoadBearing());

      return ret;
   }

   public ThighStatePacket(Random random)
   {
      this(random.nextBoolean() ? RobotSide.LEFT : RobotSide.RIGHT, random.nextBoolean() ? true : false);
   }
}
