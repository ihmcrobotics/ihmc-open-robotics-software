package isaac_ros_tensor_list_interfaces.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class Tensor extends Packet<Tensor> implements Settable<Tensor>, EpsilonComparable<Tensor>
{
   /**
            * Name of the tensor
            */
   public java.lang.StringBuilder name_;
   /**
            * Shape information for tensor
            */
   public isaac_ros_tensor_list_interfaces.msg.dds.TensorShape shape_;
   /**
            * Data type for tensor
            * Use the following values to represent these data types:
            * -  1: "int8"
            * -  2: "uint8"
            * -  3: "int16"
            * -  4: "uint16"
            * -  5: "int32"
            * -  6: "uint32"
            * -  7: "int64"
            * -  8: "uint64"
            * -  9: "float32"
            * - 10: "float64"
            */
   public int data_type_;
   /**
            * Strides of tensor (byte size for each dimension)
            */
   public us.ihmc.idl.IDLSequence.Long  strides_;
   /**
            * Data buffer
            */
   public us.ihmc.idl.IDLSequence.Byte  data_;

   public Tensor()
   {
      name_ = new java.lang.StringBuilder(255);
      shape_ = new isaac_ros_tensor_list_interfaces.msg.dds.TensorShape();
      strides_ = new us.ihmc.idl.IDLSequence.Long (100, "type_12");

      data_ = new us.ihmc.idl.IDLSequence.Byte (100, "type_9");

   }

   public Tensor(Tensor other)
   {
      this();
      set(other);
   }

   public void set(Tensor other)
   {
      name_.setLength(0);
      name_.append(other.name_);

      isaac_ros_tensor_list_interfaces.msg.dds.TensorShapePubSubType.staticCopy(other.shape_, shape_);
      data_type_ = other.data_type_;

      strides_.set(other.strides_);
      data_.set(other.data_);
   }

   /**
            * Name of the tensor
            */
   public void setName(java.lang.String name)
   {
      name_.setLength(0);
      name_.append(name);
   }

   /**
            * Name of the tensor
            */
   public java.lang.String getNameAsString()
   {
      return getName().toString();
   }
   /**
            * Name of the tensor
            */
   public java.lang.StringBuilder getName()
   {
      return name_;
   }


   /**
            * Shape information for tensor
            */
   public isaac_ros_tensor_list_interfaces.msg.dds.TensorShape getShape()
   {
      return shape_;
   }

   /**
            * Data type for tensor
            * Use the following values to represent these data types:
            * -  1: "int8"
            * -  2: "uint8"
            * -  3: "int16"
            * -  4: "uint16"
            * -  5: "int32"
            * -  6: "uint32"
            * -  7: "int64"
            * -  8: "uint64"
            * -  9: "float32"
            * - 10: "float64"
            */
   public void setDataType(int data_type)
   {
      data_type_ = data_type;
   }
   /**
            * Data type for tensor
            * Use the following values to represent these data types:
            * -  1: "int8"
            * -  2: "uint8"
            * -  3: "int16"
            * -  4: "uint16"
            * -  5: "int32"
            * -  6: "uint32"
            * -  7: "int64"
            * -  8: "uint64"
            * -  9: "float32"
            * - 10: "float64"
            */
   public int getDataType()
   {
      return data_type_;
   }


   /**
            * Strides of tensor (byte size for each dimension)
            */
   public us.ihmc.idl.IDLSequence.Long  getStrides()
   {
      return strides_;
   }


   /**
            * Data buffer
            */
   public us.ihmc.idl.IDLSequence.Byte  getData()
   {
      return data_;
   }


   public static Supplier<TensorPubSubType> getPubSubType()
   {
      return TensorPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return TensorPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(Tensor other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.name_, other.name_, epsilon)) return false;

      if (!this.shape_.epsilonEquals(other.shape_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.data_type_, other.data_type_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsLongSequence(this.strides_, other.strides_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.data_, other.data_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof Tensor)) return false;

      Tensor otherMyClass = (Tensor) other;

      if (!us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_)) return false;

      if (!this.shape_.equals(otherMyClass.shape_)) return false;
      if(this.data_type_ != otherMyClass.data_type_) return false;

      if (!this.strides_.equals(otherMyClass.strides_)) return false;
      if (!this.data_.equals(otherMyClass.data_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("Tensor {");
      builder.append("name=");
      builder.append(this.name_);      builder.append(", ");
      builder.append("shape=");
      builder.append(this.shape_);      builder.append(", ");
      builder.append("data_type=");
      builder.append(this.data_type_);      builder.append(", ");
      builder.append("strides=");
      builder.append(this.strides_);      builder.append(", ");
      builder.append("data=");
      builder.append(this.data_);
      builder.append("}");
      return builder.toString();
   }
}
