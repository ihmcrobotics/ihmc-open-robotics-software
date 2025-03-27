package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC height map module
       */
public class TerrainMapMessage extends Packet<TerrainMapMessage> implements Settable<TerrainMapMessage>, EpsilonComparable<TerrainMapMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * The number of cells per side of the terrain map grid
            */
   public int local_grid_size_;
   /**
            * The number of cells contained per meter. This defines the resolution of the grid.
            */
   public byte cells_per_meter_;
   /**
            * X coordinate of the center of the terrain map
            */
   public double map_center_x_;
   /**
            * Y coordinate of the center of the terrain map
            */
   public double map_center_y_;
   /**
            * These are the scale factors that are used to convert from the short values to the actual heights.
            * First the shorts are masked with 0xFFFF. Then they are scaled down by the scale factor and offset by the offset.
            * height = short & 0xFFFF / height_scale_factor - height_scale_offset
            */
   public double height_scale_factor_;
   public double height_scale_offset_;
   public boolean has_height_map_data_;
   public boolean has_snapped_height_data_;
   public boolean has_snapped_normal_data_;
   public boolean has_snapped_area_data_;
   public boolean has_steppability_data_;
   /**
            * The raw data for the raw heights, which are stored as shorts.
            */
   public us.ihmc.idl.IDLSequence.Byte  height_map_data_;
   /**
            * The raw data for the snapped heights, which are stored as shorts.
            */
   public us.ihmc.idl.IDLSequence.Byte  snapped_height_data_;
   /**
            * The raw data for the snap normal x value, which are stored as chars.
            */
   public us.ihmc.idl.IDLSequence.Byte  snapped_normal_x_data_;
   /**
            * The raw data for the snap normal y value, which are stored as chars.
            */
   public us.ihmc.idl.IDLSequence.Byte  snapped_normal_y_data_;
   /**
            * The raw data for the snap normal z value, which are stored as chars.
            */
   public us.ihmc.idl.IDLSequence.Byte  snapped_normal_z_data_;
   /**
            * The raw data for the snapped support area, which are stored as chars.
            */
   public us.ihmc.idl.IDLSequence.Byte  snapped_area_data_;
   /**
            * The raw data for the snapped support area, which are stored as chars.
            */
   public us.ihmc.idl.IDLSequence.Byte  steppability_data_;

   public TerrainMapMessage()
   {
      height_map_data_ = new us.ihmc.idl.IDLSequence.Byte (500000, "type_9");

      snapped_height_data_ = new us.ihmc.idl.IDLSequence.Byte (500000, "type_9");

      snapped_normal_x_data_ = new us.ihmc.idl.IDLSequence.Byte (250000, "type_9");

      snapped_normal_y_data_ = new us.ihmc.idl.IDLSequence.Byte (250000, "type_9");

      snapped_normal_z_data_ = new us.ihmc.idl.IDLSequence.Byte (250000, "type_9");

      snapped_area_data_ = new us.ihmc.idl.IDLSequence.Byte (250000, "type_9");

      steppability_data_ = new us.ihmc.idl.IDLSequence.Byte (250000, "type_9");

   }

   public TerrainMapMessage(TerrainMapMessage other)
   {
      this();
      set(other);
   }

   public void set(TerrainMapMessage other)
   {
      sequence_id_ = other.sequence_id_;

      local_grid_size_ = other.local_grid_size_;

      cells_per_meter_ = other.cells_per_meter_;

      map_center_x_ = other.map_center_x_;

      map_center_y_ = other.map_center_y_;

      height_scale_factor_ = other.height_scale_factor_;

      height_scale_offset_ = other.height_scale_offset_;

      has_height_map_data_ = other.has_height_map_data_;

      has_snapped_height_data_ = other.has_snapped_height_data_;

      has_snapped_normal_data_ = other.has_snapped_normal_data_;

      has_snapped_area_data_ = other.has_snapped_area_data_;

      has_steppability_data_ = other.has_steppability_data_;

      height_map_data_.set(other.height_map_data_);
      snapped_height_data_.set(other.snapped_height_data_);
      snapped_normal_x_data_.set(other.snapped_normal_x_data_);
      snapped_normal_y_data_.set(other.snapped_normal_y_data_);
      snapped_normal_z_data_.set(other.snapped_normal_z_data_);
      snapped_area_data_.set(other.snapped_area_data_);
      steppability_data_.set(other.steppability_data_);
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
            * The number of cells per side of the terrain map grid
            */
   public void setLocalGridSize(int local_grid_size)
   {
      local_grid_size_ = local_grid_size;
   }
   /**
            * The number of cells per side of the terrain map grid
            */
   public int getLocalGridSize()
   {
      return local_grid_size_;
   }

   /**
            * The number of cells contained per meter. This defines the resolution of the grid.
            */
   public void setCellsPerMeter(byte cells_per_meter)
   {
      cells_per_meter_ = cells_per_meter;
   }
   /**
            * The number of cells contained per meter. This defines the resolution of the grid.
            */
   public byte getCellsPerMeter()
   {
      return cells_per_meter_;
   }

   /**
            * X coordinate of the center of the terrain map
            */
   public void setMapCenterX(double map_center_x)
   {
      map_center_x_ = map_center_x;
   }
   /**
            * X coordinate of the center of the terrain map
            */
   public double getMapCenterX()
   {
      return map_center_x_;
   }

   /**
            * Y coordinate of the center of the terrain map
            */
   public void setMapCenterY(double map_center_y)
   {
      map_center_y_ = map_center_y;
   }
   /**
            * Y coordinate of the center of the terrain map
            */
   public double getMapCenterY()
   {
      return map_center_y_;
   }

   /**
            * These are the scale factors that are used to convert from the short values to the actual heights.
            * First the shorts are masked with 0xFFFF. Then they are scaled down by the scale factor and offset by the offset.
            * height = short & 0xFFFF / height_scale_factor - height_scale_offset
            */
   public void setHeightScaleFactor(double height_scale_factor)
   {
      height_scale_factor_ = height_scale_factor;
   }
   /**
            * These are the scale factors that are used to convert from the short values to the actual heights.
            * First the shorts are masked with 0xFFFF. Then they are scaled down by the scale factor and offset by the offset.
            * height = short & 0xFFFF / height_scale_factor - height_scale_offset
            */
   public double getHeightScaleFactor()
   {
      return height_scale_factor_;
   }

   public void setHeightScaleOffset(double height_scale_offset)
   {
      height_scale_offset_ = height_scale_offset;
   }
   public double getHeightScaleOffset()
   {
      return height_scale_offset_;
   }

   public void setHasHeightMapData(boolean has_height_map_data)
   {
      has_height_map_data_ = has_height_map_data;
   }
   public boolean getHasHeightMapData()
   {
      return has_height_map_data_;
   }

   public void setHasSnappedHeightData(boolean has_snapped_height_data)
   {
      has_snapped_height_data_ = has_snapped_height_data;
   }
   public boolean getHasSnappedHeightData()
   {
      return has_snapped_height_data_;
   }

   public void setHasSnappedNormalData(boolean has_snapped_normal_data)
   {
      has_snapped_normal_data_ = has_snapped_normal_data;
   }
   public boolean getHasSnappedNormalData()
   {
      return has_snapped_normal_data_;
   }

   public void setHasSnappedAreaData(boolean has_snapped_area_data)
   {
      has_snapped_area_data_ = has_snapped_area_data;
   }
   public boolean getHasSnappedAreaData()
   {
      return has_snapped_area_data_;
   }

   public void setHasSteppabilityData(boolean has_steppability_data)
   {
      has_steppability_data_ = has_steppability_data;
   }
   public boolean getHasSteppabilityData()
   {
      return has_steppability_data_;
   }


   /**
            * The raw data for the raw heights, which are stored as shorts.
            */
   public us.ihmc.idl.IDLSequence.Byte  getHeightMapData()
   {
      return height_map_data_;
   }


   /**
            * The raw data for the snapped heights, which are stored as shorts.
            */
   public us.ihmc.idl.IDLSequence.Byte  getSnappedHeightData()
   {
      return snapped_height_data_;
   }


   /**
            * The raw data for the snap normal x value, which are stored as chars.
            */
   public us.ihmc.idl.IDLSequence.Byte  getSnappedNormalXData()
   {
      return snapped_normal_x_data_;
   }


   /**
            * The raw data for the snap normal y value, which are stored as chars.
            */
   public us.ihmc.idl.IDLSequence.Byte  getSnappedNormalYData()
   {
      return snapped_normal_y_data_;
   }


   /**
            * The raw data for the snap normal z value, which are stored as chars.
            */
   public us.ihmc.idl.IDLSequence.Byte  getSnappedNormalZData()
   {
      return snapped_normal_z_data_;
   }


   /**
            * The raw data for the snapped support area, which are stored as chars.
            */
   public us.ihmc.idl.IDLSequence.Byte  getSnappedAreaData()
   {
      return snapped_area_data_;
   }


   /**
            * The raw data for the snapped support area, which are stored as chars.
            */
   public us.ihmc.idl.IDLSequence.Byte  getSteppabilityData()
   {
      return steppability_data_;
   }


   public static Supplier<TerrainMapMessagePubSubType> getPubSubType()
   {
      return TerrainMapMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return TerrainMapMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(TerrainMapMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.local_grid_size_, other.local_grid_size_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cells_per_meter_, other.cells_per_meter_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.map_center_x_, other.map_center_x_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.map_center_y_, other.map_center_y_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.height_scale_factor_, other.height_scale_factor_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.height_scale_offset_, other.height_scale_offset_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_height_map_data_, other.has_height_map_data_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_snapped_height_data_, other.has_snapped_height_data_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_snapped_normal_data_, other.has_snapped_normal_data_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_snapped_area_data_, other.has_snapped_area_data_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_steppability_data_, other.has_steppability_data_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.height_map_data_, other.height_map_data_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.snapped_height_data_, other.snapped_height_data_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.snapped_normal_x_data_, other.snapped_normal_x_data_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.snapped_normal_y_data_, other.snapped_normal_y_data_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.snapped_normal_z_data_, other.snapped_normal_z_data_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.snapped_area_data_, other.snapped_area_data_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.steppability_data_, other.steppability_data_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof TerrainMapMessage)) return false;

      TerrainMapMessage otherMyClass = (TerrainMapMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.local_grid_size_ != otherMyClass.local_grid_size_) return false;

      if(this.cells_per_meter_ != otherMyClass.cells_per_meter_) return false;

      if(this.map_center_x_ != otherMyClass.map_center_x_) return false;

      if(this.map_center_y_ != otherMyClass.map_center_y_) return false;

      if(this.height_scale_factor_ != otherMyClass.height_scale_factor_) return false;

      if(this.height_scale_offset_ != otherMyClass.height_scale_offset_) return false;

      if(this.has_height_map_data_ != otherMyClass.has_height_map_data_) return false;

      if(this.has_snapped_height_data_ != otherMyClass.has_snapped_height_data_) return false;

      if(this.has_snapped_normal_data_ != otherMyClass.has_snapped_normal_data_) return false;

      if(this.has_snapped_area_data_ != otherMyClass.has_snapped_area_data_) return false;

      if(this.has_steppability_data_ != otherMyClass.has_steppability_data_) return false;

      if (!this.height_map_data_.equals(otherMyClass.height_map_data_)) return false;
      if (!this.snapped_height_data_.equals(otherMyClass.snapped_height_data_)) return false;
      if (!this.snapped_normal_x_data_.equals(otherMyClass.snapped_normal_x_data_)) return false;
      if (!this.snapped_normal_y_data_.equals(otherMyClass.snapped_normal_y_data_)) return false;
      if (!this.snapped_normal_z_data_.equals(otherMyClass.snapped_normal_z_data_)) return false;
      if (!this.snapped_area_data_.equals(otherMyClass.snapped_area_data_)) return false;
      if (!this.steppability_data_.equals(otherMyClass.steppability_data_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("TerrainMapMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("local_grid_size=");
      builder.append(this.local_grid_size_);      builder.append(", ");
      builder.append("cells_per_meter=");
      builder.append(this.cells_per_meter_);      builder.append(", ");
      builder.append("map_center_x=");
      builder.append(this.map_center_x_);      builder.append(", ");
      builder.append("map_center_y=");
      builder.append(this.map_center_y_);      builder.append(", ");
      builder.append("height_scale_factor=");
      builder.append(this.height_scale_factor_);      builder.append(", ");
      builder.append("height_scale_offset=");
      builder.append(this.height_scale_offset_);      builder.append(", ");
      builder.append("has_height_map_data=");
      builder.append(this.has_height_map_data_);      builder.append(", ");
      builder.append("has_snapped_height_data=");
      builder.append(this.has_snapped_height_data_);      builder.append(", ");
      builder.append("has_snapped_normal_data=");
      builder.append(this.has_snapped_normal_data_);      builder.append(", ");
      builder.append("has_snapped_area_data=");
      builder.append(this.has_snapped_area_data_);      builder.append(", ");
      builder.append("has_steppability_data=");
      builder.append(this.has_steppability_data_);      builder.append(", ");
      builder.append("height_map_data=");
      builder.append(this.height_map_data_);      builder.append(", ");
      builder.append("snapped_height_data=");
      builder.append(this.snapped_height_data_);      builder.append(", ");
      builder.append("snapped_normal_x_data=");
      builder.append(this.snapped_normal_x_data_);      builder.append(", ");
      builder.append("snapped_normal_y_data=");
      builder.append(this.snapped_normal_y_data_);      builder.append(", ");
      builder.append("snapped_normal_z_data=");
      builder.append(this.snapped_normal_z_data_);      builder.append(", ");
      builder.append("snapped_area_data=");
      builder.append(this.snapped_area_data_);      builder.append(", ");
      builder.append("steppability_data=");
      builder.append(this.steppability_data_);
      builder.append("}");
      return builder.toString();
   }
}
