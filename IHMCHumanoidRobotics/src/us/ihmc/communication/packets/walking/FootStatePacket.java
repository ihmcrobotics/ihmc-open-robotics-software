package us.ihmc.communication.packets.walking;

import java.util.Random;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.robotics.robotSide.RobotSide;

@ClassDocumentation(documentation = "This message tells the controller whether a foot is load bearing.")
public class FootStatePacket extends IHMCRosApiPacket<FootStatePacket>
{
   public RobotSide robotSide;

   public boolean loadBearing;

   public FootStatePacket()
   {
      // Empty constructor for deserialization
   }

   public FootStatePacket(RobotSide robotSide, boolean loadBearing)
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
      return robotSide.getSideNameFirstLetter() + " Foot Load Bearing";
   }

   public boolean equals(Object obj)
   {
      return ((obj instanceof FootStatePacket) && this.epsilonEquals((FootStatePacket) obj, 0));
   }

   @Override
   public boolean epsilonEquals(FootStatePacket other, double epsilon)
   {
      boolean ret = (this.getRobotSide() == other.getRobotSide());
      ret &= (this.isLoadBearing() == other.isLoadBearing());

      return ret;
   }

   public FootStatePacket(Random random)
   {
      this(random.nextBoolean() ? RobotSide.LEFT : RobotSide.RIGHT, true);
   }
}
