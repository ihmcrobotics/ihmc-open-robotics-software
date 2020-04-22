package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class HeatMapPacket extends Packet<HeatMapPacket> implements Settable<HeatMapPacket>, EpsilonComparable<HeatMapPacket>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public us.ihmc.idl.IDLSequence.Float  data_;

   public int width_;

   public int height_;

   public java.lang.StringBuilder name_;

   public HeatMapPacket()
   {


      data_ = new us.ihmc.idl.IDLSequence.Float (100, "type_5");




      name_ = new java.lang.StringBuilder(255);

   }

   public HeatMapPacket(HeatMapPacket other)
   {
      this();
      set(other);
   }

   public void set(HeatMapPacket other)
   {

      sequence_id_ = other.sequence_id_;


      data_.set(other.data_);

      width_ = other.width_;


      height_ = other.height_;


      name_.setLength(0);
      name_.append(other.name_);

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



   public us.ihmc.idl.IDLSequence.Float  getData()
   {
      return data_;
   }


   public void setWidth(int width)
   {
      width_ = width;
   }
   public int getWidth()
   {
      return width_;
   }


   public void setHeight(int height)
   {
      height_ = height;
   }
   public int getHeight()
   {
      return height_;
   }


   public void setName(java.lang.String name)
   {
      name_.setLength(0);
      name_.append(name);
   }

   public java.lang.String getNameAsString()
   {
      return getName().toString();
   }
   public java.lang.StringBuilder getName()
   {
      return name_;
   }


   public static Supplier<HeatMapPacketPubSubType> getPubSubType()
   {
      return HeatMapPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return HeatMapPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(HeatMapPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.data_, other.data_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.width_, other.width_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.height_, other.height_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.name_, other.name_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof HeatMapPacket)) return false;

      HeatMapPacket otherMyClass = (HeatMapPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if (!this.data_.equals(otherMyClass.data_)) return false;

      if(this.width_ != otherMyClass.width_) return false;


      if(this.height_ != otherMyClass.height_) return false;


      if (!us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_)) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HeatMapPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("data=");
      builder.append(this.data_);      builder.append(", ");

      builder.append("width=");
      builder.append(this.width_);      builder.append(", ");

      builder.append("height=");
      builder.append(this.height_);      builder.append(", ");

      builder.append("name=");
      builder.append(this.name_);
      builder.append("}");
      return builder.toString();
   }
}
