package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ChunkMessage extends Packet<ChunkMessage> implements Settable<ChunkMessage>, EpsilonComparable<ChunkMessage>
{
   /**
            * We have a hash to keep each chunk unique
            */
   public int hash_code_of_chunk_;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * X coordinate of the center of the height map
            */
   public double origin_x_;
   /**
            * Y coordinate of the center of the height map
            */
   public double origin_y_;
   /**
            * Width of the height map in meters
            */
   public double width_in_meters_;
   /**
            * Cell size in meters of an individual cell
            */
   public double cell_size_in_meters_;
   /**
            * Cells per axis
            */
   public int cells_per_axis_;
   /**
            * List of heights, which correspond to the list of keys
            */
   public us.ihmc.idl.IDLSequence.Byte  heights_;

   public ChunkMessage()
   {
      heights_ = new us.ihmc.idl.IDLSequence.Byte (50000, "type_9");

   }

   public ChunkMessage(ChunkMessage other)
   {
      this();
      set(other);
   }

   public void set(ChunkMessage other)
   {
      hash_code_of_chunk_ = other.hash_code_of_chunk_;

      sequence_id_ = other.sequence_id_;

      origin_x_ = other.origin_x_;

      origin_y_ = other.origin_y_;

      width_in_meters_ = other.width_in_meters_;

      cell_size_in_meters_ = other.cell_size_in_meters_;

      cells_per_axis_ = other.cells_per_axis_;

      heights_.set(other.heights_);
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

   /**
            * X coordinate of the center of the height map
            */
   public void setOriginX(double origin_x)
   {
      origin_x_ = origin_x;
   }
   /**
            * X coordinate of the center of the height map
            */
   public double getOriginX()
   {
      return origin_x_;
   }

   /**
            * Y coordinate of the center of the height map
            */
   public void setOriginY(double origin_y)
   {
      origin_y_ = origin_y;
   }
   /**
            * Y coordinate of the center of the height map
            */
   public double getOriginY()
   {
      return origin_y_;
   }

   /**
            * Width of the height map in meters
            */
   public void setWidthInMeters(double width_in_meters)
   {
      width_in_meters_ = width_in_meters;
   }
   /**
            * Width of the height map in meters
            */
   public double getWidthInMeters()
   {
      return width_in_meters_;
   }

   /**
            * Cell size in meters of an individual cell
            */
   public void setCellSizeInMeters(double cell_size_in_meters)
   {
      cell_size_in_meters_ = cell_size_in_meters;
   }
   /**
            * Cell size in meters of an individual cell
            */
   public double getCellSizeInMeters()
   {
      return cell_size_in_meters_;
   }

   /**
            * Cells per axis
            */
   public void setCellsPerAxis(int cells_per_axis)
   {
      cells_per_axis_ = cells_per_axis;
   }
   /**
            * Cells per axis
            */
   public int getCellsPerAxis()
   {
      return cells_per_axis_;
   }


   /**
            * List of heights, which correspond to the list of keys
            */
   public us.ihmc.idl.IDLSequence.Byte  getHeights()
   {
      return heights_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.origin_x_, other.origin_x_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.origin_y_, other.origin_y_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.width_in_meters_, other.width_in_meters_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cell_size_in_meters_, other.cell_size_in_meters_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cells_per_axis_, other.cells_per_axis_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.heights_, other.heights_, epsilon)) return false;


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

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.origin_x_ != otherMyClass.origin_x_) return false;

      if(this.origin_y_ != otherMyClass.origin_y_) return false;

      if(this.width_in_meters_ != otherMyClass.width_in_meters_) return false;

      if(this.cell_size_in_meters_ != otherMyClass.cell_size_in_meters_) return false;

      if(this.cells_per_axis_ != otherMyClass.cells_per_axis_) return false;

      if (!this.heights_.equals(otherMyClass.heights_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ChunkMessage {");
      builder.append("hash_code_of_chunk=");
      builder.append(this.hash_code_of_chunk_);      builder.append(", ");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("origin_x=");
      builder.append(this.origin_x_);      builder.append(", ");
      builder.append("origin_y=");
      builder.append(this.origin_y_);      builder.append(", ");
      builder.append("width_in_meters=");
      builder.append(this.width_in_meters_);      builder.append(", ");
      builder.append("cell_size_in_meters=");
      builder.append(this.cell_size_in_meters_);      builder.append(", ");
      builder.append("cells_per_axis=");
      builder.append(this.cells_per_axis_);      builder.append(", ");
      builder.append("heights=");
      builder.append(this.heights_);
      builder.append("}");
      return builder.toString();
   }
}
