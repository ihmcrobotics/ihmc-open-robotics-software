package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * This message is part of the IHMC humanoid behavior module.
 */
public class SimpleCoactiveBehaviorDataPacket extends Packet<SimpleCoactiveBehaviorDataPacket>
      implements Settable<SimpleCoactiveBehaviorDataPacket>, EpsilonComparable<SimpleCoactiveBehaviorDataPacket>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   public java.lang.StringBuilder key_;
   public double value_;

   public SimpleCoactiveBehaviorDataPacket()
   {
      header_ = new std_msgs.msg.dds.Header();
      key_ = new java.lang.StringBuilder(255);
   }

   public SimpleCoactiveBehaviorDataPacket(SimpleCoactiveBehaviorDataPacket other)
   {
      this();
      set(other);
   }

   public void set(SimpleCoactiveBehaviorDataPacket other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      key_.setLength(0);
      key_.append(other.key_);

      value_ = other.value_;

   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   public void setKey(java.lang.String key)
   {
      key_.setLength(0);
      key_.append(key);
   }

   public java.lang.String getKeyAsString()
   {
      return getKey().toString();
   }

   public java.lang.StringBuilder getKey()
   {
      return key_;
   }

   public void setValue(double value)
   {
      value_ = value;
   }

   public double getValue()
   {
      return value_;
   }

   @Override
   public boolean epsilonEquals(SimpleCoactiveBehaviorDataPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.key_, other.key_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.value_, other.value_, epsilon))
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
      if (!(other instanceof SimpleCoactiveBehaviorDataPacket))
         return false;

      SimpleCoactiveBehaviorDataPacket otherMyClass = (SimpleCoactiveBehaviorDataPacket) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (!us.ihmc.idl.IDLTools.equals(this.key_, otherMyClass.key_))
         return false;

      if (this.value_ != otherMyClass.value_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SimpleCoactiveBehaviorDataPacket {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("key=");
      builder.append(this.key_);
      builder.append(", ");
      builder.append("value=");
      builder.append(this.value_);
      builder.append("}");
      return builder.toString();
   }
}
