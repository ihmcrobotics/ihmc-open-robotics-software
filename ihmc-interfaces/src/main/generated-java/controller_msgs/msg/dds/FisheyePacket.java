package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

public class FisheyePacket extends Packet<FisheyePacket> implements Settable<FisheyePacket>, EpsilonComparable<FisheyePacket>
{
   public controller_msgs.msg.dds.VideoPacket video_packet_;

   public FisheyePacket()
   {
      video_packet_ = new controller_msgs.msg.dds.VideoPacket();
   }

   public FisheyePacket(FisheyePacket other)
   {
      set(other);
   }

   public void set(FisheyePacket other)
   {
      controller_msgs.msg.dds.VideoPacketPubSubType.staticCopy(other.video_packet_, video_packet_);
   }

   public controller_msgs.msg.dds.VideoPacket getVideoPacket()
   {
      return video_packet_;
   }

   @Override
   public boolean epsilonEquals(FisheyePacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.video_packet_.epsilonEquals(other.video_packet_, epsilon))
         return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof FisheyePacket))
         return false;

      FisheyePacket otherMyClass = (FisheyePacket) other;

      if (!this.video_packet_.equals(otherMyClass.video_packet_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FisheyePacket {");
      builder.append("video_packet=");
      builder.append(this.video_packet_);

      builder.append("}");
      return builder.toString();
   }
}