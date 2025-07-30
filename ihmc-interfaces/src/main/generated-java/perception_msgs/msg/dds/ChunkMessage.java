package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Most of the data we want is actually stored inside the HeightMapMessage, so we don't have to worry about storing it here, we just need a hash
       */
public class ChunkMessage extends Packet<ChunkMessage> implements Settable<ChunkMessage>, EpsilonComparable<ChunkMessage>
{
   /**
            * We have a hash to keep each chunk unique
            */
   public int hash_code_of_chunk_;
   /**
            * How we store the data for the chunk
            */
   public perception_msgs.msg.dds.HeightMapMessage chunk_;

   public ChunkMessage()
   {
      chunk_ = new perception_msgs.msg.dds.HeightMapMessage();
   }

   public ChunkMessage(ChunkMessage other)
   {
      this();
      set(other);
   }

   public void set(ChunkMessage other)
   {
      hash_code_of_chunk_ = other.hash_code_of_chunk_;

      perception_msgs.msg.dds.HeightMapMessagePubSubType.staticCopy(other.chunk_, chunk_);
   }

   /**
            * We have a hash to keep each chunk unique
            */
   public void setHashCodeOfChunk(int hash_code_of_chunk)
   {
      hash_code_of_chunk_ = hash_code_of_chunk;
   }
   /**
            * We have a hash to keep each chunk unique
            */
   public int getHashCodeOfChunk()
   {
      return hash_code_of_chunk_;
   }


   /**
            * How we store the data for the chunk
            */
   public perception_msgs.msg.dds.HeightMapMessage getChunk()
   {
      return chunk_;
   }


   public static Supplier<ChunkMessagePubSubType> getPubSubType()
   {
      return ChunkMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ChunkMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ChunkMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.hash_code_of_chunk_, other.hash_code_of_chunk_, epsilon)) return false;

      if (!this.chunk_.epsilonEquals(other.chunk_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ChunkMessage)) return false;

      ChunkMessage otherMyClass = (ChunkMessage) other;

      if(this.hash_code_of_chunk_ != otherMyClass.hash_code_of_chunk_) return false;

      if (!this.chunk_.equals(otherMyClass.chunk_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ChunkMessage {");
      builder.append("hash_code_of_chunk=");
      builder.append(this.hash_code_of_chunk_);      builder.append(", ");
      builder.append("chunk=");
      builder.append(this.chunk_);
      builder.append("}");
      return builder.toString();
   }
}
