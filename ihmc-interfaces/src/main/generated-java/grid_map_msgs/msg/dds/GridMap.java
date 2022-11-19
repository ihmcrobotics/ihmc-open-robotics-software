package grid_map_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class GridMap extends Packet<GridMap> implements Settable<GridMap>, EpsilonComparable<GridMap>
{
   /**
            * Header (time and frame)
            */
   public std_msgs.msg.dds.Header header_;
   /**
            * Grid map header
            */
   public grid_map_msgs.msg.dds.GridMapInfo info_;
   /**
            * Grid map layer names.
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  layers_;
   /**
            * Grid map basic layer names (optional). The basic layers
            * determine which layers from `layers` need to be valid
            * in order for a cell of the grid map to be valid.
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  basic_layers_;
   /**
            * Grid map data.
            */
   public us.ihmc.idl.IDLSequence.Object<std_msgs.msg.dds.Float32MultiArray>  data_;
   /**
            * Row start index (default 0).
            */
   public int outer_start_index_;
   /**
            * Column start index (default 0).
            */
   public int inner_start_index_;

   public GridMap()
   {
      header_ = new std_msgs.msg.dds.Header();
      info_ = new grid_map_msgs.msg.dds.GridMapInfo();
      layers_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (100, "type_d");
      basic_layers_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (100, "type_d");
      data_ = new us.ihmc.idl.IDLSequence.Object<std_msgs.msg.dds.Float32MultiArray> (100, new std_msgs.msg.dds.Float32MultiArrayPubSubType());

   }

   public GridMap(GridMap other)
   {
      this();
      set(other);
   }

   public void set(GridMap other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      grid_map_msgs.msg.dds.GridMapInfoPubSubType.staticCopy(other.info_, info_);
      layers_.set(other.layers_);
      basic_layers_.set(other.basic_layers_);
      data_.set(other.data_);
      outer_start_index_ = other.outer_start_index_;

      inner_start_index_ = other.inner_start_index_;

   }


   /**
            * Header (time and frame)
            */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }


   /**
            * Grid map header
            */
   public grid_map_msgs.msg.dds.GridMapInfo getInfo()
   {
      return info_;
   }


   /**
            * Grid map layer names.
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  getLayers()
   {
      return layers_;
   }


   /**
            * Grid map basic layer names (optional). The basic layers
            * determine which layers from `layers` need to be valid
            * in order for a cell of the grid map to be valid.
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  getBasicLayers()
   {
      return basic_layers_;
   }


   /**
            * Grid map data.
            */
   public us.ihmc.idl.IDLSequence.Object<std_msgs.msg.dds.Float32MultiArray>  getData()
   {
      return data_;
   }

   /**
            * Row start index (default 0).
            */
   public void setOuterStartIndex(int outer_start_index)
   {
      outer_start_index_ = outer_start_index;
   }
   /**
            * Row start index (default 0).
            */
   public int getOuterStartIndex()
   {
      return outer_start_index_;
   }

   /**
            * Column start index (default 0).
            */
   public void setInnerStartIndex(int inner_start_index)
   {
      inner_start_index_ = inner_start_index;
   }
   /**
            * Column start index (default 0).
            */
   public int getInnerStartIndex()
   {
      return inner_start_index_;
   }


   public static Supplier<GridMapPubSubType> getPubSubType()
   {
      return GridMapPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return GridMapPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(GridMap other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon)) return false;
      if (!this.info_.epsilonEquals(other.info_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.layers_, other.layers_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.basic_layers_, other.basic_layers_, epsilon)) return false;

      if (this.data_.size() != other.data_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.data_.size(); i++)
         {  if (!this.data_.get(i).epsilonEquals(other.data_.get(i), epsilon)) return false; }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.outer_start_index_, other.outer_start_index_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.inner_start_index_, other.inner_start_index_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof GridMap)) return false;

      GridMap otherMyClass = (GridMap) other;

      if (!this.header_.equals(otherMyClass.header_)) return false;
      if (!this.info_.equals(otherMyClass.info_)) return false;
      if (!this.layers_.equals(otherMyClass.layers_)) return false;
      if (!this.basic_layers_.equals(otherMyClass.basic_layers_)) return false;
      if (!this.data_.equals(otherMyClass.data_)) return false;
      if(this.outer_start_index_ != otherMyClass.outer_start_index_) return false;

      if(this.inner_start_index_ != otherMyClass.inner_start_index_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("GridMap {");
      builder.append("header=");
      builder.append(this.header_);      builder.append(", ");
      builder.append("info=");
      builder.append(this.info_);      builder.append(", ");
      builder.append("layers=");
      builder.append(this.layers_);      builder.append(", ");
      builder.append("basic_layers=");
      builder.append(this.basic_layers_);      builder.append(", ");
      builder.append("data=");
      builder.append(this.data_);      builder.append(", ");
      builder.append("outer_start_index=");
      builder.append(this.outer_start_index_);      builder.append(", ");
      builder.append("inner_start_index=");
      builder.append(this.inner_start_index_);
      builder.append("}");
      return builder.toString();
   }
}
