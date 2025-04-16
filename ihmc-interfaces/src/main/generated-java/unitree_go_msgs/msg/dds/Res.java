package unitree_go_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class Res extends Packet<Res> implements Settable<Res>, EpsilonComparable<Res>
{
   public java.lang.StringBuilder uuid_;
   public us.ihmc.idl.IDLSequence.Byte  data_;
   public java.lang.StringBuilder body_;

   public Res()
   {
      uuid_ = new java.lang.StringBuilder(255);
      data_ = new us.ihmc.idl.IDLSequence.Byte (100, "type_9");

      body_ = new java.lang.StringBuilder(255);
   }

   public Res(Res other)
   {
      this();
      set(other);
   }

   public void set(Res other)
   {
      uuid_.setLength(0);
      uuid_.append(other.uuid_);

      data_.set(other.data_);
      body_.setLength(0);
      body_.append(other.body_);

   }

   public void setUuid(java.lang.String uuid)
   {
      uuid_.setLength(0);
      uuid_.append(uuid);
   }

   public java.lang.String getUuidAsString()
   {
      return getUuid().toString();
   }
   public java.lang.StringBuilder getUuid()
   {
      return uuid_;
   }


   public us.ihmc.idl.IDLSequence.Byte  getData()
   {
      return data_;
   }

   public void setBody(java.lang.String body)
   {
      body_.setLength(0);
      body_.append(body);
   }

   public java.lang.String getBodyAsString()
   {
      return getBody().toString();
   }
   public java.lang.StringBuilder getBody()
   {
      return body_;
   }


   public static Supplier<ResPubSubType> getPubSubType()
   {
      return ResPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ResPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(Res other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.uuid_, other.uuid_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.data_, other.data_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.body_, other.body_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof Res)) return false;

      Res otherMyClass = (Res) other;

      if (!us.ihmc.idl.IDLTools.equals(this.uuid_, otherMyClass.uuid_)) return false;

      if (!this.data_.equals(otherMyClass.data_)) return false;
      if (!us.ihmc.idl.IDLTools.equals(this.body_, otherMyClass.body_)) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("Res {");
      builder.append("uuid=");
      builder.append(this.uuid_);      builder.append(", ");
      builder.append("data=");
      builder.append(this.data_);      builder.append(", ");
      builder.append("body=");
      builder.append(this.body_);
      builder.append("}");
      return builder.toString();
   }
}
