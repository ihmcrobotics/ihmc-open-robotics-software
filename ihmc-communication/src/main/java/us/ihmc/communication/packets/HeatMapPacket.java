package us.ihmc.communication.packets;

import java.util.Arrays;

import us.ihmc.tools.ArrayTools;

/**
 *
 */
public class HeatMapPacket extends Packet<HeatMapPacket>
{
   public float[] data;
   public int width, height;
   public String name;

   public HeatMapPacket()
   {
   }

   public HeatMapPacket(HeatMapPacket other)
   {
      this.data = Arrays.copyOf(other.data, other.data.length);
      this.width = other.width;
      this.height = other.height;
      this.name = other.name;
   }

   @Override
   public void set(HeatMapPacket other)
   {
      this.data = Arrays.copyOf(other.data, other.data.length);
      this.width = other.width;
      this.height = other.height;
      this.name = other.name;
      setPacketInformation(other);
   }

   @Override
   public boolean epsilonEquals(HeatMapPacket other, double epsilon)
   {
      boolean widthEquals = other.width == this.width;
      boolean heightEquals = other.height == this.height;
      boolean nameEquals = (name == null && other.name == null) || (name != null && name.equals(other.name));

      return nameEquals && widthEquals && heightEquals && ArrayTools.deltaEquals(this.data, other.data, (float) epsilon);
   }
}
