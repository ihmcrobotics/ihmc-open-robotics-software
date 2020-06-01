package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class FisheyePacket extends Packet<FisheyePacket> implements Settable<FisheyePacket>, EpsilonComparable<FisheyePacket>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public controller_msgs.msg.dds.VideoPacket video_packet_;

   public FisheyePacket()
   {


      video_packet_ = new controller_msgs.msg.dds.VideoPacket();

   }

   public FisheyePacket(FisheyePacket other)
   {
      this();
      set(other);
   }

   public void set(FisheyePacket other)
   {

      sequence_id_ = other.sequence_id_;


      controller_msgs.msg.dds.VideoPacketPubSubType.staticCopy(other.video_packet_, video_packet_);
   }


   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }



   public controller_msgs.msg.dds.VideoPacket getVideoPacket()
   {
      return video_packet_;
   }


   public static Supplier<FisheyePacketPubSubType> getPubSubType()
   {
      return FisheyePacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FisheyePacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FisheyePacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!this.video_packet_.epsilonEquals(other.video_packet_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FisheyePacket)) return false;

      FisheyePacket otherMyClass = (FisheyePacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if (!this.video_packet_.equals(otherMyClass.video_packet_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FisheyePacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("video_packet=");
      builder.append(this.video_packet_);
      builder.append("}");
      return builder.toString();
   }
}
