package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * This message is part of the IHMC humanoid behavior module.
 */
public class SimpleCoactiveBehaviorDataPacket extends Packet<SimpleCoactiveBehaviorDataPacket>
      implements Settable<SimpleCoactiveBehaviorDataPacket>, EpsilonComparable<SimpleCoactiveBehaviorDataPacket>
{
   public java.lang.StringBuilder key_;
   public double value_;

   public SimpleCoactiveBehaviorDataPacket()
   {
      key_ = new java.lang.StringBuilder(255);
   }

   public SimpleCoactiveBehaviorDataPacket(SimpleCoactiveBehaviorDataPacket other)
   {
      set(other);
   }

   public void set(SimpleCoactiveBehaviorDataPacket other)
   {
      key_.setLength(0);
      key_.append(other.key_);

      value_ = other.value_;
   }

   public java.lang.String getKeyAsString()
   {
      return getKey().toString();
   }

   public java.lang.StringBuilder getKey()
   {
      return key_;
   }

   public void setKey(java.lang.String key)
   {
      key_.setLength(0);
      key_.append(key);
   }

   public double getValue()
   {
      return value_;
   }

   public void setValue(double value)
   {
      value_ = value;
   }

   @Override
   public boolean epsilonEquals(SimpleCoactiveBehaviorDataPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

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
      builder.append("key=");
      builder.append(this.key_);

      builder.append(", ");
      builder.append("value=");
      builder.append(this.value_);

      builder.append("}");
      return builder.toString();
   }
}
