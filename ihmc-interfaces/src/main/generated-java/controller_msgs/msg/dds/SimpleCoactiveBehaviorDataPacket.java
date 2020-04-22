package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC humanoid behavior module.
       */
public class SimpleCoactiveBehaviorDataPacket extends Packet<SimpleCoactiveBehaviorDataPacket> implements Settable<SimpleCoactiveBehaviorDataPacket>, EpsilonComparable<SimpleCoactiveBehaviorDataPacket>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public java.lang.StringBuilder key_;

   public double value_;

   public SimpleCoactiveBehaviorDataPacket()
   {


      key_ = new java.lang.StringBuilder(255);


   }

   public SimpleCoactiveBehaviorDataPacket(SimpleCoactiveBehaviorDataPacket other)
   {
      this();
      set(other);
   }

   public void set(SimpleCoactiveBehaviorDataPacket other)
   {

      sequence_id_ = other.sequence_id_;


      key_.setLength(0);
      key_.append(other.key_);


      value_ = other.value_;

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


   public static Supplier<SimpleCoactiveBehaviorDataPacketPubSubType> getPubSubType()
   {
      return SimpleCoactiveBehaviorDataPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SimpleCoactiveBehaviorDataPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SimpleCoactiveBehaviorDataPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.key_, other.key_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.value_, other.value_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SimpleCoactiveBehaviorDataPacket)) return false;

      SimpleCoactiveBehaviorDataPacket otherMyClass = (SimpleCoactiveBehaviorDataPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if (!us.ihmc.idl.IDLTools.equals(this.key_, otherMyClass.key_)) return false;


      if(this.value_ != otherMyClass.value_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SimpleCoactiveBehaviorDataPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("key=");
      builder.append(this.key_);      builder.append(", ");

      builder.append("value=");
      builder.append(this.value_);
      builder.append("}");
      return builder.toString();
   }
}
