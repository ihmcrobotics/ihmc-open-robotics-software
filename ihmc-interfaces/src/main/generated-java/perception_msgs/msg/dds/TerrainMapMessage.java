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
            * X coordinate of the center of the terrain map
            */
   public double map_center_x_;
   /**
            * Y coordinate of the center of the terrain map
            */
   public double map_center_y_;
   /**
            * The number of cells per side of the terrain map grid
            */
   public double width_in_meters_;
   /**
            * The number of cells contained per meter. This defines the resolution of the grid.
            */
   public byte cells_per_meter_;
   /**
            * Cell size in meters of an individual cell
            */
   public double cell_size_in_meters_;
   /**
            * The raw data for the terrain cost value, which are stored as chars.
            */
   public us.ihmc.idl.IDLSequence.Byte  terrain_cost_data_;
   /**
            * The raw data for the contact map value, which are stored as chars.
            */
   public us.ihmc.idl.IDLSequence.Byte  contact_map_data_;
   public us.ihmc.idl.IDLSequence.Float  heights_;
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
            * The raw data for the steppability area, which are stored as chars.
            */
   public us.ihmc.idl.IDLSequence.Byte  steppability_data_;
   /**
            * The raw data for the steppable connections value, which are stored as chars.
            */
   public us.ihmc.idl.IDLSequence.Byte  steppable_connections_data_;
   /**
            * Squared error term in the least-squares calculation when fitting normals
            */
   public us.ihmc.idl.IDLSequence.Float  squared_error_data_;

   public TerrainMapMessage()
   {
      terrain_cost_data_ = new us.ihmc.idl.IDLSequence.Byte (255000, "type_9");

      contact_map_data_ = new us.ihmc.idl.IDLSequence.Byte (255000, "type_9");

      heights_ = new us.ihmc.idl.IDLSequence.Float (255000, "type_5");

      snapped_normal_x_data_ = new us.ihmc.idl.IDLSequence.Byte (255000, "type_9");

      snapped_normal_y_data_ = new us.ihmc.idl.IDLSequence.Byte (255000, "type_9");

      snapped_normal_z_data_ = new us.ihmc.idl.IDLSequence.Byte (255000, "type_9");

      snapped_area_data_ = new us.ihmc.idl.IDLSequence.Byte (255000, "type_9");

      steppability_data_ = new us.ihmc.idl.IDLSequence.Byte (255000, "type_9");

      steppable_connections_data_ = new us.ihmc.idl.IDLSequence.Byte (255000, "type_9");

      squared_error_data_ = new us.ihmc.idl.IDLSequence.Float (255000, "type_5");

   }

   public TerrainMapMessage(TerrainMapMessage other)
   {
      this();
      set(other);
   }

   public void set(TerrainMapMessage other)
   {
      sequence_id_ = other.sequence_id_;

      map_center_x_ = other.map_center_x_;

      map_center_y_ = other.map_center_y_;

      width_in_meters_ = other.width_in_meters_;

      cells_per_meter_ = other.cells_per_meter_;

      cell_size_in_meters_ = other.cell_size_in_meters_;

      terrain_cost_data_.set(other.terrain_cost_data_);
      contact_map_data_.set(other.contact_map_data_);
      heights_.set(other.heights_);
      snapped_normal_x_data_.set(other.snapped_normal_x_data_);
      snapped_normal_y_data_.set(other.snapped_normal_y_data_);
      snapped_normal_z_data_.set(other.snapped_normal_z_data_);
      snapped_area_data_.set(other.snapped_area_data_);
      steppability_data_.set(other.steppability_data_);
      steppable_connections_data_.set(other.steppable_connections_data_);
      squared_error_data_.set(other.squared_error_data_);
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
            * The number of cells per side of the terrain map grid
            */
   public void setWidthInMeters(double width_in_meters)
   {
      width_in_meters_ = width_in_meters;
   }
   /**
            * The number of cells per side of the terrain map grid
            */
   public double getWidthInMeters()
   {
      return width_in_meters_;
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
            * The raw data for the terrain cost value, which are stored as chars.
            */
   public us.ihmc.idl.IDLSequence.Byte  getTerrainCostData()
   {
      return terrain_cost_data_;
   }


   /**
            * The raw data for the contact map value, which are stored as chars.
            */
   public us.ihmc.idl.IDLSequence.Byte  getContactMapData()
   {
      return contact_map_data_;
   }


   public us.ihmc.idl.IDLSequence.Float  getHeights()
   {
      return heights_;
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
            * The raw data for the steppability area, which are stored as chars.
            */
   public us.ihmc.idl.IDLSequence.Byte  getSteppabilityData()
   {
      return steppability_data_;
   }


   /**
            * The raw data for the steppable connections value, which are stored as chars.
            */
   public us.ihmc.idl.IDLSequence.Byte  getSteppableConnectionsData()
   {
      return steppable_connections_data_;
   }


   /**
            * Squared error term in the least-squares calculation when fitting normals
            */
   public us.ihmc.idl.IDLSequence.Float  getSquaredErrorData()
   {
      return squared_error_data_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.map_center_x_, other.map_center_x_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.map_center_y_, other.map_center_y_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.width_in_meters_, other.width_in_meters_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cells_per_meter_, other.cells_per_meter_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cell_size_in_meters_, other.cell_size_in_meters_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.terrain_cost_data_, other.terrain_cost_data_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.contact_map_data_, other.contact_map_data_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.heights_, other.heights_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.snapped_normal_x_data_, other.snapped_normal_x_data_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.snapped_normal_y_data_, other.snapped_normal_y_data_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.snapped_normal_z_data_, other.snapped_normal_z_data_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.snapped_area_data_, other.snapped_area_data_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.steppability_data_, other.steppability_data_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.steppable_connections_data_, other.steppable_connections_data_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.squared_error_data_, other.squared_error_data_, epsilon)) return false;


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

      if(this.map_center_x_ != otherMyClass.map_center_x_) return false;

      if(this.map_center_y_ != otherMyClass.map_center_y_) return false;

      if(this.width_in_meters_ != otherMyClass.width_in_meters_) return false;

      if(this.cells_per_meter_ != otherMyClass.cells_per_meter_) return false;

      if(this.cell_size_in_meters_ != otherMyClass.cell_size_in_meters_) return false;

      if (!this.terrain_cost_data_.equals(otherMyClass.terrain_cost_data_)) return false;
      if (!this.contact_map_data_.equals(otherMyClass.contact_map_data_)) return false;
      if (!this.heights_.equals(otherMyClass.heights_)) return false;
      if (!this.snapped_normal_x_data_.equals(otherMyClass.snapped_normal_x_data_)) return false;
      if (!this.snapped_normal_y_data_.equals(otherMyClass.snapped_normal_y_data_)) return false;
      if (!this.snapped_normal_z_data_.equals(otherMyClass.snapped_normal_z_data_)) return false;
      if (!this.snapped_area_data_.equals(otherMyClass.snapped_area_data_)) return false;
      if (!this.steppability_data_.equals(otherMyClass.steppability_data_)) return false;
      if (!this.steppable_connections_data_.equals(otherMyClass.steppable_connections_data_)) return false;
      if (!this.squared_error_data_.equals(otherMyClass.squared_error_data_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("TerrainMapMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("map_center_x=");
      builder.append(this.map_center_x_);      builder.append(", ");
      builder.append("map_center_y=");
      builder.append(this.map_center_y_);      builder.append(", ");
      builder.append("width_in_meters=");
      builder.append(this.width_in_meters_);      builder.append(", ");
      builder.append("cells_per_meter=");
      builder.append(this.cells_per_meter_);      builder.append(", ");
      builder.append("cell_size_in_meters=");
      builder.append(this.cell_size_in_meters_);      builder.append(", ");
      builder.append("terrain_cost_data=");
      builder.append(this.terrain_cost_data_);      builder.append(", ");
      builder.append("contact_map_data=");
      builder.append(this.contact_map_data_);      builder.append(", ");
      builder.append("heights=");
      builder.append(this.heights_);      builder.append(", ");
      builder.append("snapped_normal_x_data=");
      builder.append(this.snapped_normal_x_data_);      builder.append(", ");
      builder.append("snapped_normal_y_data=");
      builder.append(this.snapped_normal_y_data_);      builder.append(", ");
      builder.append("snapped_normal_z_data=");
      builder.append(this.snapped_normal_z_data_);      builder.append(", ");
      builder.append("snapped_area_data=");
      builder.append(this.snapped_area_data_);      builder.append(", ");
      builder.append("steppability_data=");
      builder.append(this.steppability_data_);      builder.append(", ");
      builder.append("steppable_connections_data=");
      builder.append(this.steppable_connections_data_);      builder.append(", ");
      builder.append("squared_error_data=");
      builder.append(this.squared_error_data_);
      builder.append("}");
      return builder.toString();
   }
}
