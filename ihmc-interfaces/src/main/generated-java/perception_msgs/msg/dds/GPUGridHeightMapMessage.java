package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the ETHZ's GPU Height Map Module.
       * This message contains GPU GridMap Message.
       * std_msgs/Header header
       * uint32 seq
       * time stamp
       * string frame_id
       * float64 resolution
       * float64 length_x
       * float64 length_y
       * geometry_msgs/Pose pose
       * # geometry_msgs/Point position
       * ## float64 x
       * ## float64 y
       * ## float64 z
       * # geometry_msgs/Quaternion orientation
       * ## float64 x
       * ## float64 y
       * ## float64 z
       * ## float64 w
       * std_msgs/MultiArrayLayout layout
       * # std_msgs/MultiArrayDimension[] dim
       * ## string label
       * ## uint32 size
       * ## uint32 stride
       * # uint32 data_offset
       * float32[] data
       */
public class GPUGridHeightMapMessage extends Packet<GPUGridHeightMapMessage> implements Settable<GPUGridHeightMapMessage>, EpsilonComparable<GPUGridHeightMapMessage>
{
   public grid_map_msgs.msg.dds.GridMapInfo info_;
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  layers_;
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  basic_layers_;
   public us.ihmc.idl.IDLSequence.Object<std_msgs.msg.dds.Float32MultiArray>  data_;
   public int outer_start_index_;
   public int inner_start_index_;

   public GPUGridHeightMapMessage()
   {
      info_ = new grid_map_msgs.msg.dds.GridMapInfo();
      layers_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (100, "type_d");
      basic_layers_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (100, "type_d");
      data_ = new us.ihmc.idl.IDLSequence.Object<std_msgs.msg.dds.Float32MultiArray> (100, new std_msgs.msg.dds.Float32MultiArrayPubSubType());

   }

   public GPUGridHeightMapMessage(GPUGridHeightMapMessage other)
   {
      this();
      set(other);
   }

   public void set(GPUGridHeightMapMessage other)
   {
      grid_map_msgs.msg.dds.GridMapInfoPubSubType.staticCopy(other.info_, info_);
      layers_.set(other.layers_);
      basic_layers_.set(other.basic_layers_);
      data_.set(other.data_);
      outer_start_index_ = other.outer_start_index_;

      inner_start_index_ = other.inner_start_index_;

   }


   public grid_map_msgs.msg.dds.GridMapInfo getInfo()
   {
      return info_;
   }


   public us.ihmc.idl.IDLSequence.StringBuilderHolder  getLayers()
   {
      return layers_;
   }


   public us.ihmc.idl.IDLSequence.StringBuilderHolder  getBasicLayers()
   {
      return basic_layers_;
   }


   public us.ihmc.idl.IDLSequence.Object<std_msgs.msg.dds.Float32MultiArray>  getData()
   {
      return data_;
   }

   public void setOuterStartIndex(int outer_start_index)
   {
      outer_start_index_ = outer_start_index;
   }
   public int getOuterStartIndex()
   {
      return outer_start_index_;
   }

   public void setInnerStartIndex(int inner_start_index)
   {
      inner_start_index_ = inner_start_index;
   }
   public int getInnerStartIndex()
   {
      return inner_start_index_;
   }


   public static Supplier<GPUGridHeightMapMessagePubSubType> getPubSubType()
   {
      return GPUGridHeightMapMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return GPUGridHeightMapMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(GPUGridHeightMapMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

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
      if(!(other instanceof GPUGridHeightMapMessage)) return false;

      GPUGridHeightMapMessage otherMyClass = (GPUGridHeightMapMessage) other;

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

      builder.append("GPUGridHeightMapMessage {");
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
