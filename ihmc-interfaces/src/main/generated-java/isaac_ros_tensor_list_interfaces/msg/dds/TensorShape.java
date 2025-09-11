package isaac_ros_tensor_list_interfaces.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class TensorShape extends Packet<TensorShape> implements Settable<TensorShape>, EpsilonComparable<TensorShape>
{
   /**
            * Rank of the tensor
            */
   public byte rank_;
   /**
            * Number of elements in each dimension
            */
   public us.ihmc.idl.IDLSequence.Long  dims_;

   public TensorShape()
   {
      dims_ = new us.ihmc.idl.IDLSequence.Long (100, "type_4");

   }

   public TensorShape(TensorShape other)
   {
      this();
      set(other);
   }

   public void set(TensorShape other)
   {
      rank_ = other.rank_;

      dims_.set(other.dims_);
   }

   /**
            * Rank of the tensor
            */
   public void setRank(byte rank)
   {
      rank_ = rank;
   }
   /**
            * Rank of the tensor
            */
   public byte getRank()
   {
      return rank_;
   }


   /**
            * Number of elements in each dimension
            */
   public us.ihmc.idl.IDLSequence.Long  getDims()
   {
      return dims_;
   }


   public static Supplier<TensorShapePubSubType> getPubSubType()
   {
      return TensorShapePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return TensorShapePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(TensorShape other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.rank_, other.rank_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsLongSequence(this.dims_, other.dims_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof TensorShape)) return false;

      TensorShape otherMyClass = (TensorShape) other;

      if(this.rank_ != otherMyClass.rank_) return false;

      if (!this.dims_.equals(otherMyClass.dims_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("TensorShape {");
      builder.append("rank=");
      builder.append(this.rank_);      builder.append(", ");
      builder.append("dims=");
      builder.append(this.dims_);
      builder.append("}");
      return builder.toString();
   }
}
