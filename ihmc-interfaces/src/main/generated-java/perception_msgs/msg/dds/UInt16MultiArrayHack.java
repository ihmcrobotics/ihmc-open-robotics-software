package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This is a stupid copy of the UInt16MultiArray message to get enough array elements
       * because ihmc-ros2-library defaults unbounded arrays to 100 elements.
       * TODO: jros2 Remove this message when we transition to jros2
       */
public class UInt16MultiArrayHack extends Packet<UInt16MultiArrayHack> implements Settable<UInt16MultiArrayHack>, EpsilonComparable<UInt16MultiArrayHack>
{
   /**
            * specification of data layout
            */
   public std_msgs.msg.dds.MultiArrayLayout layout_;
   /**
            * array of data
            */
   public us.ihmc.idl.IDLSequence.Integer  data_;

   public UInt16MultiArrayHack()
   {
      layout_ = new std_msgs.msg.dds.MultiArrayLayout();
      data_ = new us.ihmc.idl.IDLSequence.Integer (512, "type_3");

   }

   public UInt16MultiArrayHack(UInt16MultiArrayHack other)
   {
      this();
      set(other);
   }

   public void set(UInt16MultiArrayHack other)
   {
      std_msgs.msg.dds.MultiArrayLayoutPubSubType.staticCopy(other.layout_, layout_);
      data_.set(other.data_);
   }


   /**
            * specification of data layout
            */
   public std_msgs.msg.dds.MultiArrayLayout getLayout()
   {
      return layout_;
   }


   /**
            * array of data
            */
   public us.ihmc.idl.IDLSequence.Integer  getData()
   {
      return data_;
   }


   public static Supplier<UInt16MultiArrayHackPubSubType> getPubSubType()
   {
      return UInt16MultiArrayHackPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return UInt16MultiArrayHackPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(UInt16MultiArrayHack other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.layout_.epsilonEquals(other.layout_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.data_, other.data_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof UInt16MultiArrayHack)) return false;

      UInt16MultiArrayHack otherMyClass = (UInt16MultiArrayHack) other;

      if (!this.layout_.equals(otherMyClass.layout_)) return false;
      if (!this.data_.equals(otherMyClass.data_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("UInt16MultiArrayHack {");
      builder.append("layout=");
      builder.append(this.layout_);      builder.append(", ");
      builder.append("data=");
      builder.append(this.data_);
      builder.append("}");
      return builder.toString();
   }
}
