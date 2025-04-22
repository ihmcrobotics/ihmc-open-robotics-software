package unitree_go_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class Error extends Packet<Error> implements Settable<Error>, EpsilonComparable<Error>
{
   public long error_source_;
   public long error_state_;

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
      error_source_ = other.error_source_;

      error_state_ = other.error_state_;

   }

   public void setErrorSource(long error_source)
   {
      error_source_ = error_source;
   }
   public long getErrorSource()
   {
      return error_source_;
   }

   public void setErrorState(long error_state)
   {
      error_state_ = error_state;
   }
   public long getErrorState()
   {
      return error_state_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.error_source_, other.error_source_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.error_state_, other.error_state_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof Error)) return false;

      Error otherMyClass = (Error) other;

      if(this.error_source_ != otherMyClass.error_source_) return false;

      if(this.error_state_ != otherMyClass.error_state_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("Error {");
      builder.append("error_source=");
      builder.append(this.error_source_);      builder.append(", ");
      builder.append("error_state=");
      builder.append(this.error_state_);
      builder.append("}");
      return builder.toString();
   }
}
