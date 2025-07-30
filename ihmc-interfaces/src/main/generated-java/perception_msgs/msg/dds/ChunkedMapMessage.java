package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ChunkedMapMessage extends Packet<ChunkedMapMessage> implements Settable<ChunkedMapMessage>, EpsilonComparable<ChunkedMapMessage>
{
   /**
            * Contains all the chunks to created the map (chunked map)
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.ChunkMessage>  chunks_;

   public ChunkedMapMessage()
   {
      chunks_ = new us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.ChunkMessage> (100, new perception_msgs.msg.dds.ChunkMessagePubSubType());

   }

   public ChunkedMapMessage(ChunkedMapMessage other)
   {
      this();
      set(other);
   }

   public void set(ChunkedMapMessage other)
   {
      chunks_.set(other.chunks_);
   }


   /**
            * Contains all the chunks to created the map (chunked map)
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.ChunkMessage>  getChunks()
   {
      return chunks_;
   }


   public static Supplier<ChunkedMapMessagePubSubType> getPubSubType()
   {
      return ChunkedMapMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ChunkedMapMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ChunkedMapMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (this.chunks_.size() != other.chunks_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.chunks_.size(); i++)
         {  if (!this.chunks_.get(i).epsilonEquals(other.chunks_.get(i), epsilon)) return false; }
      }

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ChunkedMapMessage)) return false;

      ChunkedMapMessage otherMyClass = (ChunkedMapMessage) other;

      if (!this.chunks_.equals(otherMyClass.chunks_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ChunkedMapMessage {");
      builder.append("chunks=");
      builder.append(this.chunks_);
      builder.append("}");
      return builder.toString();
   }
}
