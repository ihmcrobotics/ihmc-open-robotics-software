package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ChunkMessage extends Packet<ChunkMessage> implements Settable<ChunkMessage>, EpsilonComparable<ChunkMessage>
{
   /**
            * This is the location in world of the center of the chunk
            */
   public double center_x_;
   public double center_y_;
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
      center_x_ = other.center_x_;

      center_y_ = other.center_y_;

      hash_code_of_chunk_ = other.hash_code_of_chunk_;

      perception_msgs.msg.dds.HeightMapMessagePubSubType.staticCopy(other.chunk_, chunk_);
   }

   /**
            * This is the location in world of the center of the chunk
            */
   public void setCenterX(double center_x)
   {
      center_x_ = center_x;
   }
   /**
            * This is the location in world of the center of the chunk
            */
   public double getCenterX()
   {
      return center_x_;
   }

   public void setCenterY(double center_y)
   {
      center_y_ = center_y;
   }
   public double getCenterY()
   {
      return center_y_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.center_x_, other.center_x_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.center_y_, other.center_y_, epsilon)) return false;

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

      if(this.center_x_ != otherMyClass.center_x_) return false;

      if(this.center_y_ != otherMyClass.center_y_) return false;

      if(this.hash_code_of_chunk_ != otherMyClass.hash_code_of_chunk_) return false;

      if (!this.chunk_.equals(otherMyClass.chunk_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ChunkMessage {");
      builder.append("center_x=");
      builder.append(this.center_x_);      builder.append(", ");
      builder.append("center_y=");
      builder.append(this.center_y_);      builder.append(", ");
      builder.append("hash_code_of_chunk=");
      builder.append(this.hash_code_of_chunk_);      builder.append(", ");
      builder.append("chunk=");
      builder.append(this.chunk_);
      builder.append("}");
      return builder.toString();
   }
}
