package unitree_go_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class Error extends Packet<Error> implements Settable<Error>, EpsilonComparable<Error>
{
   public long source_;
   public long state_;

   public Error()
   {
   }

   public Error(Error other)
   {
      this();
      set(other);
   }

   public void set(Error other)
   {
      source_ = other.source_;

      state_ = other.state_;

   }

   public void setSource(long source)
   {
      source_ = source;
   }
   public long getSource()
   {
      return source_;
   }

   public void setState(long state)
   {
      state_ = state;
   }
   public long getState()
   {
      return state_;
   }


   public static Supplier<ErrorPubSubType> getPubSubType()
   {
      return ErrorPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ErrorPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(Error other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.source_, other.source_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.state_, other.state_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof Error)) return false;

      Error otherMyClass = (Error) other;

      if(this.source_ != otherMyClass.source_) return false;

      if(this.state_ != otherMyClass.state_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("Error {");
      builder.append("source=");
      builder.append(this.source_);      builder.append(", ");
      builder.append("state=");
      builder.append(this.state_);
      builder.append("}");
      return builder.toString();
   }
}
