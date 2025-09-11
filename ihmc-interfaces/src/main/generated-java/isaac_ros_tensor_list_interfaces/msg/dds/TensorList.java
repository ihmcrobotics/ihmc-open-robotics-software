package isaac_ros_tensor_list_interfaces.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class TensorList extends Packet<TensorList> implements Settable<TensorList>, EpsilonComparable<TensorList>
{
   public std_msgs.msg.dds.Header header_;
   /**
            * A list of tensors
            */
   public us.ihmc.idl.IDLSequence.Object<isaac_ros_tensor_list_interfaces.msg.dds.Tensor>  tensors_;

   public TensorList()
   {
      header_ = new std_msgs.msg.dds.Header();
      tensors_ = new us.ihmc.idl.IDLSequence.Object<isaac_ros_tensor_list_interfaces.msg.dds.Tensor> (100, new isaac_ros_tensor_list_interfaces.msg.dds.TensorPubSubType());

   }

   public TensorList(TensorList other)
   {
      this();
      set(other);
   }

   public void set(TensorList other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      tensors_.set(other.tensors_);
   }


   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }


   /**
            * A list of tensors
            */
   public us.ihmc.idl.IDLSequence.Object<isaac_ros_tensor_list_interfaces.msg.dds.Tensor>  getTensors()
   {
      return tensors_;
   }


   public static Supplier<TensorListPubSubType> getPubSubType()
   {
      return TensorListPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return TensorListPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(TensorList other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon)) return false;
      if (this.tensors_.size() != other.tensors_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.tensors_.size(); i++)
         {  if (!this.tensors_.get(i).epsilonEquals(other.tensors_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof TensorList)) return false;

      TensorList otherMyClass = (TensorList) other;

      if (!this.header_.equals(otherMyClass.header_)) return false;
      if (!this.tensors_.equals(otherMyClass.tensors_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("TensorList {");
      builder.append("header=");
      builder.append(this.header_);      builder.append(", ");
      builder.append("tensors=");
      builder.append(this.tensors_);
      builder.append("}");
      return builder.toString();
   }
}
