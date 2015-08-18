package us.ihmc.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandLoadBearingPacket extends Packet<HandLoadBearingPacket>
{
   public RobotSide robotSide;
   public boolean loadBearing;

   public HandLoadBearingPacket()
   {
      // Empty constructor for deserialization
   }

   public HandLoadBearingPacket(RobotSide robotSide, boolean loadBearing)
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

   public String toString()
   {
      if (loadBearing)
         return robotSide.getSideNameFirstLetter() + " Hand Load Bearing";
      else
         return robotSide.getSideNameFirstLetter() + " Hand not Load Bearing";
   }

   public boolean equals(Object obj)
   {
      return ((obj instanceof HandLoadBearingPacket) && this.epsilonEquals((HandLoadBearingPacket) obj, 0));
   }

   @Override
   public boolean epsilonEquals(HandLoadBearingPacket other, double epsilon)
   {
      boolean ret = this.getRobotSide().equals(other.getRobotSide());
      ret &= (this.isLoadBearing() == other.isLoadBearing());

      return ret;
   }

   public HandLoadBearingPacket(Random random)
   {
      this(random.nextBoolean() ? RobotSide.LEFT : RobotSide.RIGHT, true);
   }
}
