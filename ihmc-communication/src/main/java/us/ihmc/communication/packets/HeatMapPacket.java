package us.ihmc.communication.packets;

import gnu.trove.list.array.TFloatArrayList;
import us.ihmc.tools.ArrayTools;

/**
 *
 */
public class HeatMapPacket extends Packet<HeatMapPacket>
{
   public TFloatArrayList data = new TFloatArrayList();
   public int width, height;
   public StringBuilder name = new StringBuilder();

   public HeatMapPacket()
   {
   }

   public HeatMapPacket(HeatMapPacket other)
   {
      MessageTools.copyData(other.data, data);
      this.width = other.width;
      this.height = other.height;
      this.name.append(other.name);
   }

   @Override
   public void set(HeatMapPacket other)
   {
      MessageTools.copyData(other.data, data);
      this.width = other.width;
      this.height = other.height;
      this.name.setLength(0);
      this.name.append(other.name);
      setPacketInformation(other);
   }

   @Override
   public boolean epsilonEquals(HeatMapPacket other, double epsilon)
   {
      boolean widthEquals = other.width == this.width;
      boolean heightEquals = other.height == this.height;
      boolean nameEquals = (name == null && other.name == null) || (name != null && name.equals(other.name));

      return nameEquals && widthEquals && heightEquals && ArrayTools.deltaEquals(this.data.toArray(), other.data.toArray(), (float) epsilon);
   }
}
