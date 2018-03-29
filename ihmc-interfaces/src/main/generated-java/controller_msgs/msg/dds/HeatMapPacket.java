package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

public class HeatMapPacket extends Packet<HeatMapPacket> implements Settable<HeatMapPacket>, EpsilonComparable<HeatMapPacket>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   public us.ihmc.idl.IDLSequence.Float data_;
   public int width_;
   public int height_;
   public java.lang.StringBuilder name_;

   public HeatMapPacket()
   {
      header_ = new std_msgs.msg.dds.Header();
      data_ = new us.ihmc.idl.IDLSequence.Float(100, "type_5");

      name_ = new java.lang.StringBuilder(255);
   }

   public HeatMapPacket(HeatMapPacket other)
   {
      this();
      set(other);
   }

   public void set(HeatMapPacket other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      data_.set(other.data_);
      width_ = other.width_;

      height_ = other.height_;

      name_.setLength(0);
      name_.append(other.name_);

   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   public us.ihmc.idl.IDLSequence.Float getData()
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

   @Override
   public boolean epsilonEquals(HeatMapPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.data_, other.data_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.width_, other.width_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.height_, other.height_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.name_, other.name_, epsilon))
         return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof HeatMapPacket))
         return false;

      HeatMapPacket otherMyClass = (HeatMapPacket) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (!this.data_.equals(otherMyClass.data_))
         return false;
      if (this.width_ != otherMyClass.width_)
         return false;

      if (this.height_ != otherMyClass.height_)
         return false;

      if (!us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HeatMapPacket {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("data=");
      builder.append(this.data_);
      builder.append(", ");
      builder.append("width=");
      builder.append(this.width_);
      builder.append(", ");
      builder.append("height=");
      builder.append(this.height_);
      builder.append(", ");
      builder.append("name=");
      builder.append(this.name_);
      builder.append("}");
      return builder.toString();
   }
}
